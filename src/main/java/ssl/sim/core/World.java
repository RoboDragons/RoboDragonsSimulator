package ssl.sim.core;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import ssl.sim.ball.Ball;
import ssl.sim.ball.BallContacts;
import ssl.sim.physics.Collisions;
import ssl.sim.physics.ConvexPolygon;
import ssl.sim.robot.Robot;
import ssl.sim.robot.RobotSpecs;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.locks.ReentrantLock;

/** Simulation world: field, ball and robots. All physics is advanced with a fixed sub-step. */
public class World {
    public static final double PHYSICS_DT = 0.0005;

    private FieldGeometry field;
    private final Ball ball;
    private final List<Robot> robots = new CopyOnWriteArrayList<>();
    private final ReentrantLock lock = new ReentrantLock();
    private final BallContacts.Material wallMaterial;
    private double time;
    private double accumulator;

    // robot contact parameters
    private double robotStiffness = 2.0e5, robotDampingRatio = 0.3, robotMu = 0.5;

    public World(FieldGeometry field) { this(field, new Ball.Params()); }

    public World(FieldGeometry field, Ball.Params ballParams) {
        this.field = field;
        this.ball = new Ball(ballParams);
        this.wallMaterial = new BallContacts.Material(1.0e5, 0.25, ballParams.muWall);
    }

    public ReentrantLock lock() { return lock; }
    public FieldGeometry field() { return field; }
    public void setField(FieldGeometry f) { this.field = f; }
    public Ball ball() { return ball; }
    public List<Robot> robots() { return robots; }
    public double time() { return time; }

    public Robot addRobot(Robot.Team team, int id, RobotSpecs specs, double x, double y, double heading) {
        removeRobot(team, id);
        Robot r = new Robot(team, id, specs);
        r.setPose(x, y, heading);
        robots.add(r);
        return r;
    }

    public void removeRobot(Robot.Team team, int id) {
        robots.removeIf(r -> r.team == team && r.id == id);
    }

    public Robot findRobot(Robot.Team team, int id) {
        for (Robot r : robots) if (r.team == team && r.id == id) return r;
        return null;
    }

    /** Advance simulated time by {@code seconds} using fixed physics sub-steps. */
    public void advance(double seconds) {
        accumulator += seconds;
        while (accumulator >= PHYSICS_DT - 1e-12) {
            step(PHYSICS_DT);
            accumulator -= PHYSICS_DT;
        }
    }

    public void step(double dt) {
        time += dt;
        for (Robot r : robots) r.updateComponents(ball, time, dt);
        for (Robot r : robots) r.collideBall(ball, dt);
        collideBallWalls(dt);
        collideRobots(dt);
        for (Robot r : robots) r.integratePhysics(dt);
        ball.step(dt);
        enforceBounds();
    }

    private void collideBallWalls(double dt) {
        if (ball.z() - ball.radius() >= field.wallHeight) return;
        for (ConvexPolygon wall : field.collisionWalls()) {
            Collisions.Contact c = Collisions.circleVsPolygon(ball.position2D(), ball.radius(), wall);
            if (c == null) continue;
            Vector3D n = new Vector3D(c.normal().getX(), c.normal().getY(), 0);
            BallContacts.resolve(ball, n, c.depth(), Vector3D.ZERO, wallMaterial, dt);
        }
    }

    private void collideRobots(double dt) {
        int n = robots.size();
        for (int i = 0; i < n; i++) {
            Robot a = robots.get(i);
            ConvexPolygon pa = null;
            for (int j = i + 1; j < n; j++) {
                Robot b = robots.get(j);
                double rr = a.specs().radius + b.specs().radius;
                if (a.position().distanceSq(b.position()) > rr * rr) continue;
                if (pa == null) pa = a.hullWorld();
                Collisions.Contact c = Collisions.polygonVsPolygon(pa, b.hullWorld());
                if (c == null) continue;
                Vector2D va = a.velocityAt(c.point()), vb = b.velocityAt(c.point());
                double mRed = a.mass() * b.mass() / (a.mass() + b.mass());
                applyPenalty(a, b, c, vb.subtract(va), mRed, dt);
            }
            ConvexPolygon hull = pa != null ? pa : a.hullWorld();
            for (ConvexPolygon wall : field.collisionWalls()) {
                Collisions.Contact c = Collisions.polygonVsPolygon(hull, wall);
                if (c == null) continue;
                applyPenalty(a, null, c, a.velocityAt(c.point()).negate(), a.mass(), dt);
            }
        }
    }

    /**
     * Spring-damper normal force plus Coulomb friction; {@code b} == null means a static obstacle.
     * Friction is applied as an impulse limited by what removes the tangential slip (using the effective inverse
     * mass of the contact incl. rotation); a plain viscous-regularised friction force is numerically unstable
     * at the lever arm of the robot radius and made robots explode on wall impact.
     */
    private void applyPenalty(Robot a, Robot b, Collisions.Contact c, Vector2D relVel, double mRed, double dt) {
        Vector2D n = c.normal();
        double vn = relVel.dotProduct(n);
        double damp = 2 * robotDampingRatio * Math.sqrt(robotStiffness * mRed);
        double fn = Math.max(0, robotStiffness * c.depth() - damp * vn);
        Vector2D vt = relVel.subtract(n.scalarMultiply(vn));
        double sp = vt.getNorm();
        Vector2D f = n.scalarMultiply(fn);
        if (sp > 1e-9 && fn > 0) {
            Vector2D t = vt.scalarMultiply(1.0 / sp);
            double inv = invEffective(a, c.point(), t) + (b != null ? invEffective(b, c.point(), t) : 0);
            double j = Math.min(robotMu * fn * dt, sp / inv);
            f = f.subtract(t.scalarMultiply(j / dt));
        }
        // f acts on b (towards b along +n); a receives the opposite
        a.applyForceAt(c.point(), f.negate());
        if (b != null) b.applyForceAt(c.point(), f);
    }

    private static double invEffective(Robot r, Vector2D p, Vector2D dir) {
        double lever = (p.getX() - r.x()) * dir.getY() - (p.getY() - r.y()) * dir.getX();
        return 1.0 / r.mass() + lever * lever / r.inertia();
    }

    /** Last-resort guard: nothing can ever leave the outer boundary, whatever the contact forces do. */
    private void enforceBounds() {
        double mx = field.halfLength() + field.boundaryGoalLine, my = field.halfWidth() + field.boundaryTouchLine;
        for (Robot r : robots) {
            double x = Math.max(-mx, Math.min(mx, r.x())), y = Math.max(-my, Math.min(my, r.y()));
            if (x != r.x() || y != r.y()) {
                double vx = r.velocity().getX(), vy = r.velocity().getY();
                if (x != r.x()) vx = 0;
                if (y != r.y()) vy = 0;
                r.setPose(x, y, r.heading());
                r.setVelocity(vx, vy, r.angularVelocity());
            }
        }
        double rb = ball.radius();
        ball.constrainTo(-mx + rb, mx - rb, -my + rb, my - rb);
    }

    /** Whether a robot-sized disc at (x,y) would overlap an existing robot. */
    public boolean isFree(double x, double y, double radius) {
        for (Robot r : robots) if (r.position().distance(new Vector2D(x, y)) < radius + r.specs().radius) return false;
        return true;
    }

    public List<Robot> robots(Robot.Team team) {
        List<Robot> out = new ArrayList<>();
        for (Robot r : robots) if (r.team == team) out.add(r);
        return out;
    }
}
