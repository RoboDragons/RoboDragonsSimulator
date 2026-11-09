package ssl.sim.robot;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import ssl.sim.ball.Ball;
import ssl.sim.ball.BallContacts;

/**
 * Dribbler device: a rubber-coated roller (cylinder along the robot's y axis) driven by its own motor and
 * speed controller. A positive speed spins the roller so that its front surface moves downward, which
 * imparts backspin on the ball and pulls it towards the robot (the ground friction acting on the
 * backspinning ball does the rest).
 * <p>
 * Physics modelled per step:
 * <ul>
 *   <li>roller/ball penalty contact (spring-damper) in the plane perpendicular to the roller axis,</li>
 *   <li>Coulomb friction on the full 3D slip velocity of the contact point, which includes the ball's spin,
 *       the roller surface speed and the robot's own motion,</li>
 *   <li>equal and opposite force on the robot (force and yaw torque) and friction load torque on the
 *       roller motor (speed droop under load).</li>
 * </ul>
 */
public class Dribbler implements RobotComponent {
    public BallContacts.Material rubber = new BallContacts.Material(3.0e4, 0.8, 2.0);

    private final DCMotor motor = DCMotor.dribblerMotor();
    private final PiController controller = new PiController(0.02, 0.4, motor.maxVoltage);
    private final RobotSpecs specs;
    private final double radius, nominalAxisX, axisZ, halfLength;
    private double slide, slideVel; // rearward retraction of the roller from its nominal position [m], [m/s]
    private double omega;       // roller angular velocity about the robot's +y axis [rad/s]
    private double targetRpm;
    private boolean contact;
    private double contactNormalForce;
    private double rollerInertia = 5.0e-6;

    public Dribbler(RobotSpecs s) {
        this.radius = s.rollerRadius;
        this.specs = s;
        this.nominalAxisX = s.rollerAxisX();
        this.axisZ = s.rollerAxisZ();
        this.halfLength = s.rollerHalfLength;
    }

    public double targetRpm() { return targetRpm; }
    public void setTargetRpm(double rpm) { this.targetRpm = Math.max(0, rpm); }
    public double rollerSpeedRpm() { return omega * 60 / (2 * Math.PI); }
    public double rollerAngularVelocity() { return omega; }
    public boolean hasBallContact() { return contact; }
    public double contactNormalForce() { return contactNormalForce; }
    public double rollerRadius() { return radius; }
    /** Current x of the roller axis (moves when the damper is active). */
    public double axisX() { return nominalAxisX - slide; }
    public double slide() { return slide; }
    public double axisZ() { return axisZ; }
    public double halfLength() { return halfLength; }

    @Override
    public void update(Robot robot, Ball ball, double dt) {
        double target = targetRpm * 2 * Math.PI / 60.0;
        double volts = controller.update(target - omega, dt);
        double motorTorque = motor.torque(volts, omega);
        double ballTorque = 0;
        contact = false;
        contactNormalForce = 0;

        double axisX = nominalAxisX - slide;
        double forceOnRollerRear = 0; // ball -> roller force along the slide axis, rearward positive
        Vector2D bl = robot.toLocal(ball.position2D());
        double dx = bl.getX() - axisX, dz = ball.z() - axisZ;
        double dist = Math.hypot(dx, dz);
        double depth = ball.radius() + radius - dist;
        if (Math.abs(bl.getY()) <= halfLength && depth > 0 && dist > 1e-9) {
            double nlx = dx / dist, nlz = dz / dist;
            Vector2D nxy = robot.rotateToWorld(nlx, 0);
            Vector3D n = new Vector3D(nxy.getX(), nxy.getY(), nlz);
            Vector2D axisPoint = robot.toWorld(axisX, bl.getY());
            Vector2D contactXY = axisPoint.add(nxy.scalarMultiply(radius));
            Vector3D axisDir = new Vector3D(-Math.sin(robot.heading()), Math.cos(robot.heading()), 0);
            Vector3D rc = n.scalarMultiply(radius);
            Vector2D rv = robot.velocityAt(contactXY);
            Vector2D slideWorld = robot.rotateToWorld(-slideVel, 0);
            Vector3D surfaceVel = new Vector3D(rv.getX() + slideWorld.getX(), rv.getY() + slideWorld.getY(), 0)
                    .add(axisDir.scalarMultiply(omega).crossProduct(rc));

            Vector3D f = BallContacts.resolve(ball, n, depth, surfaceVel, rubber, dt);
            if (specs.rollerDamperEnabled) {
                // axial part goes through the slider (and its spring/damper) to the robot, lateral part directly
                Vector2D fxy = new Vector2D(f.getX(), f.getY());
                double fx = fxy.dotProduct(robot.rotateToWorld(1, 0)), fy = fxy.dotProduct(robot.rotateToWorld(0, 1));
                forceOnRollerRear = fx;
                robot.applyForceAt(contactXY, robot.rotateToWorld(0, -fy));
            } else {
                robot.applyForceAt(contactXY, new Vector2D(-f.getX(), -f.getY()));
            }
            ballTorque = rc.crossProduct(f.negate()).dotProduct(axisDir);
            contact = true;
            contactNormalForce = f.dotProduct(n);
            ball.markTouched(robot, robot.time());
        }
        if (specs.rollerDamperEnabled) stepSlider(robot, axisX, forceOnRollerRear, dt);
        double j = rollerInertia + motor.rotorInertia;
        omega += (motorTorque + ballTorque) / j * dt;
    }

    /** Spring (with preload, resting on the nominal stop) + damper + end stops between robot and roller. */
    private void stepSlider(Robot robot, double axisX, double fBallRear, double dt) {
        double m = specs.rollerSlideMass;
        double spring = -(specs.rollerPreload + specs.rollerSpringK * slide); // robot -> roller, rearward positive
        double damper = -specs.rollerDampingC * slideVel;
        double a = (fBallRear + spring + damper) / m;
        double v0 = slideVel;
        slideVel += a * dt;
        slide += slideVel * dt;
        if (slide < 0) { // nominal stop: the preload holds the roller here
            slide = 0;
            if (slideVel < 0) slideVel = 0;
        } else if (slide > specs.rollerStroke) { // fully retracted
            slide = specs.rollerStroke;
            if (slideVel > 0) slideVel = 0;
        }
        double aActual = (slideVel - v0) / dt;
        double forceRobotOnRoller = m * aActual - fBallRear; // rearward positive, includes stop forces
        // reaction on the robot acts forward (+x) with the same magnitude
        robot.applyForceAt(robot.toWorld(axisX, 0), robot.rotateToWorld(forceRobotOnRoller, 0));
    }
}
