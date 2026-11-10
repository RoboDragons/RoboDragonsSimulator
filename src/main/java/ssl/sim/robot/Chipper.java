package ssl.sim.robot;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import ssl.sim.ball.Ball;
import ssl.sim.ball.BallContacts;

/**
 * Chip kick plate (ramp) below the dribbler roller. Passively it supports the held ball: together with the
 * roller and the ground it forms the "three point" hold; with {@code RobotSpecs.chipperEnabled = false}
 * only roller and ground touch the ball ("two point" hold).
 * <p>
 * Geometry (from {@link RobotSpecs}): a flat plate in the robot's x-z plane from the low front tip to the high
 * rear end, rising towards the back, width = notch width. Contact with the ball is a penalty contact against
 * the closest point of that segment (face or edge), with Coulomb friction on the slip velocity including spin.
 * The chip kick itself (impulse along the commanded angle) is triggered through {@link Kicker}.
 */
public class Chipper implements RobotComponent {
    public BallContacts.Material plate;

    private final RobotSpecs specs;
    private final double[] tip, top;
    private final double halfWidth;
    private boolean contact;
    private double normalForce;

    public Chipper(RobotSpecs specs) {
        this.specs = specs;
        this.plate = specs.chipperMaterial;
        this.tip = specs.chipperTip();
        this.top = specs.chipperTop();
        this.halfWidth = specs.notchHalfWidth - 0.003;
    }

    public boolean enabled() { return specs.chipperEnabled; }
    public boolean hasBallContact() { return contact; }
    public double contactNormalForce() { return normalForce; }
    /** Front (low) end (x, z) in the robot frame. */
    public double[] tip() { return tip.clone(); }
    /** Rear (high) end (x, z) in the robot frame. */
    public double[] top() { return top.clone(); }
    public double halfWidth() { return halfWidth; }
    public BallContacts.Material material() { return plate; }

    @Override
    public void update(Robot robot, Ball ball, double dt) {
        contact = false;
        normalForce = 0;
        if (!specs.chipperEnabled || robot.kicker().strokeActive()) return; // during a kick the moving plate takes over
        Vector2D bl = robot.toLocal(ball.position2D());
        if (Math.abs(bl.getY()) > halfWidth + ball.radius() * 0.5) return;
        // closest point of the plate segment to the ball centre, in the x-z plane
        double ax = tip[0], az = tip[1], bx = top[0], bz = top[1];
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
        Vector3D f = BallContacts.resolve(ball, n, depth, new Vector3D(rv.getX(), rv.getY(), 0), plate, dt);
        robot.applyForceAt(cp, new Vector2D(-f.getX(), -f.getY()));
        contact = true;
        normalForce = f.dotProduct(n);
        ball.markTouched(robot, robot.time());
    }
}
