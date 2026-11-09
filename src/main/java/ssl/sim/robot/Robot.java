package ssl.sim.robot;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import ssl.sim.ball.Ball;
import ssl.sim.ball.BallContacts;
import ssl.sim.physics.Collisions;
import ssl.sim.physics.ConvexPolygon;
import ssl.sim.physics.RigidBody2D;

import java.util.ArrayList;
import java.util.List;

/**
 * One SSL robot: a planar rigid body composed of separate parts - {@link OmniDrive} (wheels + motors),
 * {@link Kicker} and {@link Dribbler}. Parts are {@link RobotComponent}s so new ones can be added.
 */
public class Robot extends RigidBody2D {
    public enum Team { YELLOW, BLUE }

    public final Team team;
    public final int id;
    private final RobotSpecs specs;
    private final RobotShape shape;
    private final OmniDrive drive;
    private final Kicker kicker;
    private final Dribbler dribbler;
    private final Chipper chipper;
    private final List<RobotComponent> components = new ArrayList<>();
    private final BallContacts.Material bodyMaterial = new BallContacts.Material(5.0e4, 0.3, 0.5);
    private double time;

    public Robot(Team team, int id, RobotSpecs specs) {
        this.team = team;
        this.id = id;
        this.specs = specs;
        this.mass = specs.mass;
        this.inertia = specs.effectiveInertia();
        this.shape = new RobotShape(specs);
        this.drive = new OmniDrive(specs);
        this.kicker = new Kicker(specs);
        this.dribbler = new Dribbler(specs);
        this.chipper = new Chipper(specs);
        components.add(drive);
        components.add(dribbler);
        components.add(chipper);
        components.add(kicker);
    }

    public RobotSpecs specs() { return specs; }
    public RobotShape shape() { return shape; }
    public OmniDrive drive() { return drive; }
    public Kicker kicker() { return kicker; }
    public Dribbler dribbler() { return dribbler; }
    public Chipper chipper() { return chipper; }
    public double time() { return time; }
    public void addComponent(RobotComponent c) { components.add(c); }

    public void applyCommand(RobotCommand cmd) {
        drive.setMove(cmd.move(), time);
        dribbler.setTargetRpm(cmd.dribblerRpm());
        if (cmd.kickSpeed() > 0) kicker.request(cmd.kickSpeed(), cmd.kickAngleDeg());
    }

    public ConvexPolygon hullWorld() { return shape.hull().transform(x, y, theta); }

    /** Actuators of the robot act on the robot and the ball. */
    public void updateComponents(Ball ball, double time, double dt) {
        this.time = time;
        for (RobotComponent c : components) c.update(this, ball, dt);
    }

    /** Contact between the ball and the robot body (excluding the dribbler roller). */
    public void collideBall(Ball ball, double dt) {
        double reach = specs.radius + ball.radius() + 0.02;
        if (ball.position2D().distance(position()) > reach) return;
        if (ball.z() - ball.radius() >= specs.height) return;
        for (ConvexPolygon piece : shape.ballPieces()) {
            ConvexPolygon p = piece.transform(x, y, theta);
            Collisions.Contact c = Collisions.circleVsPolygon(ball.position2D(), ball.radius(), p);
            if (c == null) continue;
            Vector3D n = new Vector3D(c.normal().getX(), c.normal().getY(), 0);
            Vector2D sv = velocityAt(c.point());
            Vector3D f = BallContacts.resolve(ball, n, c.depth(), new Vector3D(sv.getX(), sv.getY(), 0), bodyMaterial, dt);
            applyForceAt(c.point(), new Vector2D(-f.getX(), -f.getY()));
            ball.markTouched(this, time);
        }
    }

    public void integratePhysics(double dt) { integrate(dt); }
}
