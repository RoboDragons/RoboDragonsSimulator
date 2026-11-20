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
        return resolve(ball, n, depth, surfaceVel, mat, dt, null, 0);
    }

    /**
     * Same, for a surface that is itself driven by a body with finite inertia (a motor-driven roller): the friction impulse that
     * removes the slip is shared between the ball and that body. {@code coupledDir} is the unit direction along which the surface
     * can move (the roller's rotation direction) and {@code coupledInvMass} its effective inverse mass at the contact
     * (r^2 / J for a roller of radius r and inertia J). With a large friction coefficient the contact then sticks (rolls without slip)
     * and the load shows up as a speed drop of the driven body.
     */
    public static Vector3D resolve(Ball ball, Vector3D n, double depth, Vector3D surfaceVel, Material mat, double dt,
                                   Vector3D coupledDir, double coupledInvMass) {
        final double R = ball.radius(), m = ball.mass(), I = ball.inertia();
        Vector3D rc = n.scalarMultiply(-R); // contact point relative to ball centre
        double vn = ball.velocity().subtract(surfaceVel).dotProduct(n);
        // All contacts act on the same ball state within a step. If each of N simultaneous contacts removed all of "its" slip /
        // velocity, the sum would over-correct (e.g. a ball squeezed between two opposite rollers would flip its spin every step and
        // finally be thrown out). So every contact only takes a 1/N share of the correction: a ball held by opposing rollers ends with
        // the mean of their surface speeds (about no spin) and slips on the weaker one instead of oscillating.
        ball.registerContact();
        int shared = ball.sharedContacts();
        double c = 2 * mat.dampingRatio() * Math.sqrt(mat.stiffness() * m);
        c /= Math.max(1.0, shared * c * dt / m); // the explicit dampers of all contacts together reverse the velocity at most once
        double fn = Math.max(0, mat.stiffness() * depth - c * vn);

        Vector3D rel = ball.surfaceVelocity(rc).subtract(surfaceVel);
        Vector3D vt = rel.subtract(n.scalarMultiply(rel.dotProduct(n)));
        double sp = vt.getNorm();
        Vector3D force = n.scalarMultiply(fn);
        if (sp > 1e-9 && fn > 0) {
            double meffInv = 1.0 / m + R * R / I;
            if (coupledDir != null) {
                double along = vt.dotProduct(coupledDir) / sp;
                meffInv += coupledInvMass * along * along;
            }
            // friction is bounded by the elastic part of the normal force: the velocity dependent damper part must not feed
            // back into the tangential force (a wedge-like contact would then show negative damping)
            double fnFric = Math.min(fn, mat.stiffness() * depth);
            double j = Math.min(mat.mu() * fnFric * dt, sp / meffInv / shared);
            force = force.add(vt.scalarMultiply(-j / dt / sp));
        }
        ball.applyForce(force, rc);
        return force;
    }
}
