package ssl.sim.robot;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import ssl.sim.ball.Ball;
import ssl.sim.ball.BallContacts;

/**
 * Solenoid kicker (straight) and chip kicker built as a moving metal plate.
 * <p>
 * A capacitor bank stores the energy: a kick of speed v costs (v / vMax)^2 of the full charge and recharges linearly.
 * A kick is a short plate stroke: the plate accelerates uniformly to its final speed over {@code kickerStroke} and pushes
 * the ball through a penalty contact with <b>Coulomb friction between the metal plate and the ball</b>. The friction acts on
 * the full slip velocity of the contact point (ball spin included), so the backspin a spinning dribbler put on the ball is
 * largely wiped out while the plate pushes: after a straight kick the ball no longer comes back even with the dribbler on.
 * <ul>
 *   <li>straight kick: a vertical plate at ball-centre height moving along the robot's x axis;</li>
 *   <li>chip kick: a ramp (the chipper plate) tilted so that its normal points along the commanded elevation angle,
 *       moving along that direction.</li>
 * </ul>
 */
public class Kicker implements RobotComponent {
    public double chargeTime = 1.5;          // [s] from empty to full
    public double plateHalfWidth;            // [m], derived from the dribbler notch width
    public double minKickDelay = 0.02;
    /** Metal plate against the ball: stiff, strongly damped (little rebound), friction from RobotSpecs.kickerFriction. */
    public BallContacts.Material plate;
    /** Length of the chip ramp during the stroke [m]. */
    public double chipPlateLength = 0.03;
    /** Empirical factor so that the ball leaves with the commanded speed (penalty contact losses). */
    public double speedCalibration = 1.0;

    private final RobotSpecs specs;
    private double charge = 1.0;
    private double pendingSpeed, pendingAngle;
    private boolean pending;
    private double lastKickTime = Double.NEGATIVE_INFINITY;
    private double lastKickSpeed;
    private boolean lastKickChip;

    // stroke state
    private boolean stroking;
    private boolean strokeChip;
    private double vPlate, strokeLen, travel, speedNow, dirX, dirZ, endTimer;

    public Kicker(RobotSpecs specs) {
        this.specs = specs;
        this.plateHalfWidth = specs.notchHalfWidth - 0.003;
        this.plate = specs.kickerMaterial;
    }

    /** Straight kicker plate geometry (robot frame) for display: rear face x, centre height, size. */
    public double plateX() { return specs.kickerFrontX() - specs.kickerPlateThickness; }
    public double plateHalfWidth() { return plateHalfWidth; }
    public double plateCenterZ() { return specs.kickerCenterZ(); }
    public double plateHeight() { return specs.kickerPlateHeight; }
    public double plateThickness() { return specs.kickerPlateThickness; }
    /** 1 right after a kick, fading to 0 over {@code duration} seconds. */
    public double kickFlash(double now, double duration) { return Math.max(0, 1 - (now - lastKickTime) / duration); }
    public boolean hasPendingKick() { return pending; }
    public boolean strokeActive() { return stroking; }
    /** Distance the plate has moved forward during the current stroke [m]. */
    public double plateTravel() { return travel; }

    public double charge() { return charge; }
    public double lastKickTime() { return lastKickTime; }
    public double lastKickSpeed() { return lastKickSpeed; }
    public boolean lastKickWasChip() { return lastKickChip; }

    /** Queue a kick; it is fired on the next update if enough energy is available. */
    public void request(double speed, double angleDeg) {
        if (speed <= 0) return;
        pending = true;
        pendingSpeed = speed;
        pendingAngle = angleDeg;
    }

    /** Restitution of the plate/ball contact, used to choose the plate speed that gives the commanded ball speed. */
    private double restitution() {
        double z = plate.dampingRatio();
        return Math.exp(-Math.PI * z / Math.sqrt(1 - z * z));
    }

    @Override
    public void update(Robot robot, Ball ball, double dt) {
        charge = Math.min(1.0, charge + dt / chargeTime);
        if (!stroking && pending) tryStart(robot);
        if (stroking) stroke(robot, ball, dt);
    }

    private void tryStart(Robot robot) {
        boolean chip = pendingAngle > 0.5;
        double vMax = chip ? specs.maxChipKickSpeed : specs.maxLinearKickSpeed;
        double speed = Math.min(pendingSpeed, vMax);
        double cost = (speed / vMax) * (speed / vMax);
        if (charge < cost || robot.time() - lastKickTime < minKickDelay) return;
        pending = false;
        charge -= cost;
        lastKickTime = robot.time();
        lastKickSpeed = speed;
        lastKickChip = chip;
        stroking = true;
        strokeChip = chip;
        double a = chip ? Math.toRadians(Math.min(pendingAngle, 89)) : 0;
        dirX = Math.cos(a);
        dirZ = Math.sin(a);
        vPlate = speed / (1 + restitution()) * speedCalibration;
        strokeLen = specs.kickerStroke;
        travel = 0;
        speedNow = 0;
        endTimer = 0;
    }

    private void stroke(Robot robot, Ball ball, double dt) {
        if (travel < strokeLen) {
            double acc = vPlate * vPlate / (2 * strokeLen);
            speedNow = Math.min(vPlate, speedNow + acc * dt);
            travel += speedNow * dt;
        } else {
            speedNow = 0;
            endTimer += dt;
            if (endTimer > 0.01) { // the plate has stopped; the ball has left
                stroking = false;
                travel = 0;
                return;
            }
        }
        // plate segment in the robot's x-z plane (front face), displaced by the travel along the stroke direction
        double ax, az, bx, bz;
        if (!strokeChip) {
            double x0 = specs.kickerFrontX() + travel;
            ax = bx = x0;
            az = specs.kickerCenterZ() - specs.kickerPlateHeight / 2;
            bz = specs.kickerCenterZ() + specs.kickerPlateHeight / 2;
        } else {
            double[] pc = specs.chipperContact(); // ramp through the usual contact point, normal along the stroke direction
            double tx = -dirZ, tz = dirX;          // tangent, pointing up/back
            double e = 0.004;
            ax = pc[0] + dirX * travel - e * tx;
            az = Math.max(0.001, pc[1] + dirZ * travel - e * tz);
            bx = pc[0] + dirX * travel + chipPlateLength * tx;
            bz = pc[1] + dirZ * travel + chipPlateLength * tz;
        }
        Vector2D bl = robot.toLocal(ball.position2D());
        if (Math.abs(bl.getY()) > plateHalfWidth + ball.radius() * 0.5) return;
        double ex = bx - ax, ez = bz - az;
        double t = ((bl.getX() - ax) * ex + (ball.z() - az) * ez) / (ex * ex + ez * ez);
        t = Math.max(0, Math.min(1, t));
        double qx = ax + t * ex, qz = az + t * ez;
        double dx = bl.getX() - qx, dz = ball.z() - qz;
        double dist = Math.hypot(dx, dz);
        double depth = ball.radius() - dist;
        if (depth <= 0 || dist < 1e-9) return;
        Vector2D nxy = robot.rotateToWorld(dx / dist, 0);
        Vector3D n = new Vector3D(nxy.getX(), nxy.getY(), dz / dist);
        Vector2D cp = robot.toWorld(qx, bl.getY());
        Vector2D rv = robot.velocityAt(cp);
        Vector2D pv = robot.rotateToWorld(speedNow * dirX, 0);
        Vector3D surface = new Vector3D(rv.getX() + pv.getX(), rv.getY() + pv.getY(), speedNow * dirZ);
        Vector3D f = BallContacts.resolve(ball, n, depth, surface, plate, dt);
        robot.applyForceAt(cp, new Vector2D(-f.getX(), -f.getY()));
        ball.markTouched(robot, robot.time());
    }
}
