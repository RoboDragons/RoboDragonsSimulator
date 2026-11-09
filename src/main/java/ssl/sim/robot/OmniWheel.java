package ssl.sim.robot;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import ssl.sim.ball.Ball;

/**
 * Omni wheel with its motor, gearbox and speed controller.
 * <p>
 * The wheel transmits force along its driving direction through a Coulomb traction model (limited by
 * mu * normal load). The small free-rolling rollers are replaced by a lateral sliding model: the wheel
 * slides along its axis with a small viscous/Coulomb resistance.
 */
public class OmniWheel {
    /** Position angle (CCW from the robot's front) of the wheel on the chassis. */
    public final double mountAngle;
    public final double mountRadius;
    public final double wheelRadius;
    public final double gearRatio;
    public final DCMotor motor;
    public double wheelInertia = 1.0e-4;
    public double muTraction = 1.3;
    /** Lateral resistance (rollers) [N s/m]. */
    public double lateralDamping = 0.3;
    public double slipRegularisation = 0.03;

    private final PiController controller;
    private double omega; // wheel angular velocity [rad/s], positive = CCW driving direction
    private double lastForce;

    public OmniWheel(double mountAngle, double mountRadius, double wheelRadius, double gearRatio, DCMotor motor) {
        this.mountAngle = mountAngle;
        this.mountRadius = mountRadius;
        this.wheelRadius = wheelRadius;
        this.gearRatio = gearRatio;
        this.motor = motor;
        this.controller = new PiController(0.5, 6.0, motor.maxVoltage);
    }

    public double angularVelocity() { return omega; }

    /** Set the wheel speed from the body motion (rolling without slip); used by the ER-Force drive model. */
    public void setKinematic(double surfaceSpeed) { omega = surfaceSpeed / wheelRadius; }
    public double surfaceSpeed() { return omega * wheelRadius; }
    public double lastTractionForce() { return lastForce; }

    /** Unit vector in robot frame along which a positive wheel speed pushes the robot. */
    public Vector2D driveDirectionLocal() {
        return new Vector2D(-Math.sin(mountAngle), Math.cos(mountAngle));
    }

    /** Wheel surface speed produced by a body-frame velocity (vx, vy, w) with the wheel sliding freely. */
    public double surfaceSpeedFor(double vx, double vy, double w) {
        return -Math.sin(mountAngle) * vx + Math.cos(mountAngle) * vy + mountRadius * w;
    }

    public void update(Robot robot, double targetSurfaceSpeed, int wheelCount, double dt) {
        double c = Math.cos(mountAngle), s = Math.sin(mountAngle);
        Vector2D pw = robot.toWorld(mountRadius * c, mountRadius * s);
        Vector2D dw = robot.rotateToWorld(-s, c);
        Vector2D lw = robot.rotateToWorld(c, s);
        Vector2D vg = robot.velocityAt(pw);
        double vAlong = vg.dotProduct(dw), vLat = vg.dotProduct(lw);

        double err = targetSurfaceSpeed / wheelRadius - omega;
        double volts = controller.update(err, dt);
        double tauWheel = motor.torque(volts, omega * gearRatio) * gearRatio * 0.9;

        double jRef = wheelInertia + gearRatio * gearRatio * motor.rotorInertia;
        double normal = robot.mass() * Ball.G / wheelCount;
        double slip = omega * wheelRadius - vAlong;
        double fMax = muTraction * normal;
        double mInv = 1.0 / robot.mass() + wheelRadius * wheelRadius / jRef;
        double f = fMax * Math.max(-1, Math.min(1, slip / slipRegularisation));
        double fZero = Math.abs(slip) / (dt * mInv);
        if (Math.abs(f) > fZero) f = Math.signum(f) * fZero;
        lastForce = f;

        omega += (tauWheel - f * wheelRadius) / jRef * dt;

        double fLat = -Math.max(-fMax, Math.min(fMax, lateralDamping * vLat));
        robot.applyForceAt(pw, dw.scalarMultiply(f).add(lw.scalarMultiply(fLat)));
    }
}
