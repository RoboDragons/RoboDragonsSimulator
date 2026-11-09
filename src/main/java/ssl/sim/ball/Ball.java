package ssl.sim.ball;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import ssl.sim.robot.Robot;

/**
 * Golf-ball sized sphere (SSL rules: 43 mm diameter, 46 g) with full 3D translation and spin.
 * Ground contact uses Coulomb friction on the slip velocity of the contact point, so sliding, rolling
 * and backspin behave consistently (two-phase rolling emerges from the physics: slide until slip = 0,
 * then roll with rolling resistance). Impacts with the ground use restitution plus tangential impulse.
 */
public class Ball {
    public static final double G = 9.81;

    /** Tunable physical constants. */
    public static class Params {
        public double radius = 0.0215;
        public double mass = 0.046;
        /** I = inertiaFactor * m * r^2 (0.4 solid sphere, 0.667 thin shell). */
        public double inertiaFactor = 0.4;
        public double muSlide = 0.4;
        public double aRoll = 0.35;
        public double spinFriction = 4.0;
        public double restitutionGround = 0.5;
        public double muImpact = 0.35;
        public double airDrag = 0.0;
        /** Ball-wall friction coefficient; kept small so that a dribbler can peel the ball off a wall. */
        public double muWall = 0.3;
    }

    public final Params params;
    private double x, y, z, vx, vy, vz, wx, wy, wz;
    private double fx, fy, fz, tx, ty, tz;
    private Robot lastTouchedBy;
    private double lastTouchTime = Double.NEGATIVE_INFINITY;

    public Ball() { this(new Params()); }

    public Ball(Params p) {
        this.params = p;
        this.z = p.radius;
    }

    public double radius() { return params.radius; }
    public double mass() { return params.mass; }
    public double inertia() { return params.inertiaFactor * params.mass * params.radius * params.radius; }

    public Vector3D position() { return new Vector3D(x, y, z); }
    public Vector2D position2D() { return new Vector2D(x, y); }
    public Vector3D velocity() { return new Vector3D(vx, vy, vz); }
    public Vector3D angularVelocity() { return new Vector3D(wx, wy, wz); }
    public double x() { return x; }
    public double y() { return y; }
    public double z() { return z; }
    public boolean onGround() { return z <= params.radius + 1e-5 && vz <= 1e-3; }

    public Robot lastTouchedBy() { return lastTouchedBy; }
    public double lastTouchTime() { return lastTouchTime; }

    public void markTouched(Robot r, double time) {
        lastTouchedBy = r;
        lastTouchTime = time;
    }

    /** Teleport. If {@code roll} the spin is set so the ball rolls without slipping. */
    public void teleport(Vector3D p, Vector3D v, boolean roll) {
        x = p.getX();
        y = p.getY();
        z = Math.max(p.getZ(), params.radius);
        vx = v.getX();
        vy = v.getY();
        vz = v.getZ();
        wx = wy = wz = 0;
        if (roll) setRolling();
        fx = fy = fz = tx = ty = tz = 0;
    }

    /** Keep the centre inside the rectangle; the velocity component pointing outwards is removed, spin is kept. */
    public void constrainTo(double minX, double maxX, double minY, double maxY) {
        if (x < minX) { x = minX; if (vx < 0) vx = 0; }
        if (x > maxX) { x = maxX; if (vx > 0) vx = 0; }
        if (y < minY) { y = minY; if (vy < 0) vy = 0; }
        if (y > maxY) { y = maxY; if (vy > 0) vy = 0; }
    }

    public void setVelocity(Vector3D v) {
        vx = v.getX();
        vy = v.getY();
        vz = v.getZ();
    }

    public void setAngularVelocity(Vector3D w) {
        wx = w.getX();
        wy = w.getY();
        wz = w.getZ();
    }

    public void addVelocity(Vector3D dv) {
        vx += dv.getX();
        vy += dv.getY();
        vz += dv.getZ();
    }

    private void setRolling() {
        wy = vx / params.radius;
        wx = -vy / params.radius;
    }

    /** Contact force on the ball applied at offset r from its centre (generates torque r x F). */
    public void applyForce(Vector3D f, Vector3D r) {
        fx += f.getX();
        fy += f.getY();
        fz += f.getZ();
        tx += r.getY() * f.getZ() - r.getZ() * f.getY();
        ty += r.getZ() * f.getX() - r.getX() * f.getZ();
        tz += r.getX() * f.getY() - r.getY() * f.getX();
    }

    /** Velocity of the ball surface point at offset r from the centre. */
    public Vector3D surfaceVelocity(Vector3D r) {
        return new Vector3D(vx, vy, vz).add(new Vector3D(wx, wy, wz).crossProduct(r));
    }

    public void step(double dt) {
        final double R = params.radius, m = params.mass, I = inertia();
        final double meffInv = 1.0 / m + R * R / I;
        boolean grounded = onGround();
        double az;
        if (grounded) {
            double n = Math.max(0, m * G - fz);
            double slipX = vx - wy * R, slipY = vy + wx * R;
            double sp = Math.hypot(slipX, slipY);
            if (sp > 1e-9) {
                double j = Math.min(params.muSlide * n * dt, sp / meffInv);
                double gfx = -slipX / sp * j / dt, gfy = -slipY / sp * j / dt;
                fx += gfx;
                fy += gfy;
                tx += R * gfy;
                ty += -R * gfx;
            }
            tz -= params.spinFriction * I * wz * (n > 0 ? 1 : 0);
            az = Math.max(0, fz - m * G) / m;
        } else {
            az = fz / m - G;
        }
        if (params.airDrag > 0 && !grounded) {
            double sp = Math.sqrt(vx * vx + vy * vy + vz * vz);
            fx -= params.airDrag * sp * vx;
            fy -= params.airDrag * sp * vy;
            az -= params.airDrag * sp * vz / m;
        }
        vx += fx / m * dt;
        vy += fy / m * dt;
        vz += az * dt;
        wx += tx / I * dt;
        wy += ty / I * dt;
        wz += tz / I * dt;
        x += vx * dt;
        y += vy * dt;
        z += vz * dt;
        fx = fy = fz = tx = ty = tz = 0;

        if (z <= R) {
            z = R;
            if (vz < 0) {
                if (-vz > 0.3) bounce(meffInv);
                else vz = 0;
            }
        }
        if (onGround()) rollingResistance(dt);
    }

    private void bounce(double meffInv) {
        final double R = params.radius, m = params.mass, I = inertia();
        double vin = -vz;
        vz = params.restitutionGround * vin;
        double jn = m * (1 + params.restitutionGround) * vin;
        double slipX = vx - wy * R, slipY = vy + wx * R;
        double sp = Math.hypot(slipX, slipY);
        if (sp > 1e-9) {
            double j = Math.min(params.muImpact * jn, sp / meffInv);
            double jx = -slipX / sp * j, jy = -slipY / sp * j;
            vx += jx / m;
            vy += jy / m;
            wx += R * jy / I;
            wy += -R * jx / I;
        }
    }

    private void rollingResistance(double dt) {
        final double R = params.radius;
        double slipX = vx - wy * R, slipY = vy + wx * R;
        if (Math.hypot(slipX, slipY) < 2e-3) {
            double v = Math.hypot(vx, vy);
            double nv = Math.max(0, v - params.aRoll * dt);
            if (v > 1e-9) {
                vx *= nv / v;
                vy *= nv / v;
            }
            if (nv < 1e-3) vx = vy = 0;
            setRolling();
        }
    }
}
