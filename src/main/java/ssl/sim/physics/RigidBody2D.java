package ssl.sim.physics;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

/** Planar rigid body (x, y, heading) with force accumulators and semi-implicit Euler integration. */
public abstract class RigidBody2D {
    protected double x, y, theta;
    protected double vx, vy, omega;
    protected double mass = 1.0, inertia = 1.0;
    private double fx, fy, torque;

    public Vector2D position() { return new Vector2D(x, y); }
    public double x() { return x; }
    public double y() { return y; }
    public double heading() { return theta; }
    public Vector2D velocity() { return new Vector2D(vx, vy); }
    public double angularVelocity() { return omega; }
    public double mass() { return mass; }
    public double inertia() { return inertia; }

    public void setPose(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public void setVelocity(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    public Vector2D toWorld(double lx, double ly) {
        double c = Math.cos(theta), s = Math.sin(theta);
        return new Vector2D(x + c * lx - s * ly, y + s * lx + c * ly);
    }

    public Vector2D toLocal(Vector2D w) {
        double dx = w.getX() - x, dy = w.getY() - y;
        double c = Math.cos(theta), s = Math.sin(theta);
        return new Vector2D(c * dx + s * dy, -s * dx + c * dy);
    }

    public Vector2D rotateToWorld(double lx, double ly) {
        double c = Math.cos(theta), s = Math.sin(theta);
        return new Vector2D(c * lx - s * ly, s * lx + c * ly);
    }

    public Vector2D velocityAt(Vector2D p) {
        return new Vector2D(vx - omega * (p.getY() - y), vy + omega * (p.getX() - x));
    }

    public void applyForce(Vector2D f) {
        fx += f.getX();
        fy += f.getY();
    }

    public void applyForceAt(Vector2D p, Vector2D f) {
        fx += f.getX();
        fy += f.getY();
        torque += (p.getX() - x) * f.getY() - (p.getY() - y) * f.getX();
    }

    public void applyTorque(double t) { torque += t; }

    protected void integrate(double dt) {
        vx += fx / mass * dt;
        vy += fy / mass * dt;
        omega += torque / inertia * dt;
        x += vx * dt;
        y += vy * dt;
        theta += omega * dt;
        if (theta > Math.PI) theta -= 2 * Math.PI;
        else if (theta < -Math.PI) theta += 2 * Math.PI;
        fx = fy = torque = 0;
    }
}
