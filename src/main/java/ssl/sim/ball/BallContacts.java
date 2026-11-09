package ssl.sim.ball;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/** Penalty (spring-damper) contact between the ball and another surface, including Coulomb friction with spin. */
public final class BallContacts {
    private BallContacts() {}

    /** Contact material parameters. */
    public record Material(double stiffness, double dampingRatio, double mu) {}

    public static final Material HARD = new Material(1.0e5, 0.2, 0.5);

    /**
     * Resolves one contact and applies the force to the ball.
     *
     * @param n          unit normal pointing from the other body towards the ball centre
     * @param depth      penetration depth (> 0)
     * @param surfaceVel velocity of the other body's surface at the contact point
     * @return total force applied on the ball (the reaction on the other body is the negative)
     */
    public static Vector3D resolve(Ball ball, Vector3D n, double depth, Vector3D surfaceVel, Material mat, double dt) {
        final double R = ball.radius(), m = ball.mass(), I = ball.inertia();
        Vector3D rc = n.scalarMultiply(-R); // contact point relative to ball centre
        double vn = ball.velocity().subtract(surfaceVel).dotProduct(n);
        double c = 2 * mat.dampingRatio() * Math.sqrt(mat.stiffness() * m);
        double fn = Math.max(0, mat.stiffness() * depth - c * vn);

        Vector3D rel = ball.surfaceVelocity(rc).subtract(surfaceVel);
        Vector3D vt = rel.subtract(n.scalarMultiply(rel.dotProduct(n)));
        double sp = vt.getNorm();
        Vector3D force = n.scalarMultiply(fn);
        if (sp > 1e-9 && fn > 0) {
            double meffInv = 1.0 / m + R * R / I;
            double j = Math.min(mat.mu() * fn * dt, sp / meffInv);
            force = force.add(vt.scalarMultiply(-j / dt / sp));
        }
        ball.applyForce(force, rc);
        return force;
    }
}
