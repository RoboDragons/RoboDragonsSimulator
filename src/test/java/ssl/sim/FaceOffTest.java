package ssl.sim;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.jupiter.api.Test;
import ssl.sim.core.FieldGeometry;
import ssl.sim.core.World;
import ssl.sim.robot.*;

import static org.junit.jupiter.api.Assertions.*;

/** Two robots with spinning dribblers contest a ball head on: the ball must stay put and the robots push each other less than they hold. */
class FaceOffTest {
    private static final String[] MODELS = {"ER_FORCE", "MOTOR"};

    private static void cmd(Robot r, double forward) {
        r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(forward, 0, 0), 0, 0, 9000));
    }

    /** Both robots drive at the loose ball in the middle with their dribblers on, then keep pushing against each other. */
    @Test
    void ballBetweenTwoDribblersStaysPut() {
        for (String model : MODELS) {
            for (double v : new double[]{0.5, 1.0}) {
                RobotSpecs sa = new RobotSpecs(), sb = new RobotSpecs();
                sa.driveModel = sb.driveModel = model;
                World w = new World(new FieldGeometry(FieldGeometry.Division.A));
                Robot a = w.addRobot(Robot.Team.BLUE, 0, sa, -0.8, 0, 0);
                Robot b = w.addRobot(Robot.Team.YELLOW, 0, sb, 0.8, 0, Math.PI);
                w.ball().teleport(new Vector3D(0, 0, 0.0215), Vector3D.ZERO, false);
                double maxSpeed = 0, maxZ = 0;
                boolean touched = false;
                for (int i = 0; i < 10000; i++) {
                    if (i % 20 == 0) {
                        cmd(a, i < 4000 ? v : 0.3);
                        cmd(b, i < 4000 ? v : 0.3);
                    }
                    w.step(World.PHYSICS_DT);
                    touched |= a.dribbler().hasBallContact() && b.dribbler().hasBallContact();
                    if (touched) {
                        maxSpeed = Math.max(maxSpeed, w.ball().velocity().getNorm());
                        maxZ = Math.max(maxZ, w.ball().z());
                    }
                }
                String what = model + " v=" + v;
                var ball = w.ball();
                assertTrue(touched, what + ": both dribblers must reach the ball");
                // the ball is not flung out: never faster than the robots, not lifted, and it ends up held by one of them
                assertTrue(maxSpeed < 1.2, what + ": ball speed " + maxSpeed);
                assertTrue(maxZ < ball.radius() + 0.005, what + ": ball lifted to " + maxZ);
                double held = Math.min(ball.position2D().distance(a.toWorld(sa.holdX(), 0)), ball.position2D().distance(b.toWorld(sb.holdX(), 0)));
                assertTrue(held < 0.02, what + ": ball is " + held + " m off the dribblers");
                // no spin-up by one roller: the opposing rollers cancel (or the ball slips on one of them)
                assertTrue(ball.angularVelocity().getNorm() < 100, what + ": ball spin " + ball.angularVelocity().getNorm());
                assertTrue(ball.velocity().getNorm() < 0.5, what + ": ball speed at the end " + ball.velocity().getNorm());
            }
        }
    }

    /** The robots are not exactly aligned: the ball between them must not make them spin or slide past each other. */
    @Test
    void offsetRobotsDoNotSlidePastEachOther() {
        for (String model : MODELS) {
            for (double off : new double[]{0.005, 0.01}) {
                RobotSpecs sa = new RobotSpecs(), sb = new RobotSpecs();
                sa.driveModel = sb.driveModel = model;
                World w = new World(new FieldGeometry(FieldGeometry.Division.A));
                Robot a = w.addRobot(Robot.Team.BLUE, 0, sa, -0.8, 0, 0);
                Robot b = w.addRobot(Robot.Team.YELLOW, 0, sb, 0.8, off, Math.PI);
                w.ball().teleport(new Vector3D(0, off / 2, 0.0215), Vector3D.ZERO, false);
                double maxDy = 0, maxTurn = 0;
                for (int i = 0; i < 10000; i++) {
                    if (i % 20 == 0) {
                        cmd(a, i < 4000 ? 1.0 : 0.3);
                        cmd(b, i < 4000 ? 1.0 : 0.3);
                    }
                    w.step(World.PHYSICS_DT);
                    maxDy = Math.max(maxDy, Math.abs(a.y() - b.y()));
                    maxTurn = Math.max(maxTurn, Math.abs(a.heading()));
                }
                String what = model + " offset=" + off;
                assertTrue(maxDy < 0.15, what + ": the robots slid sideways by " + maxDy + " m");
                assertTrue(maxTurn < 1.0, what + ": the robot turned by " + maxTurn + " rad");
                assertTrue(w.ball().position2D().distance(a.toWorld(sa.holdX(), 0)) < 0.03, what + ": the ball got lost");
            }
        }
    }

    /** A robot commanded to stand still is not pushed back by one that drives against it (holding force > pushing force). */
    @Test
    void robotStaysPutWhenPushed() {
        for (String model : MODELS) {
            RobotSpecs sa = new RobotSpecs(), sb = new RobotSpecs();
            sa.driveModel = sb.driveModel = model;
            World w = new World(new FieldGeometry(FieldGeometry.Division.A));
            Robot a = w.addRobot(Robot.Team.BLUE, 0, sa, 0, 0, 0);
            Robot b = w.addRobot(Robot.Team.YELLOW, 0, sb, 1.0, 0, Math.PI);
            w.ball().teleport(new Vector3D(-3, 3, 0.0215), Vector3D.ZERO, false);
            double ax = Double.NaN;
            for (int i = 0; i < 10000; i++) {
                if (i % 20 == 0) {
                    cmd(a, 0);
                    cmd(b, 0.5);
                }
                w.step(World.PHYSICS_DT);
                if (i == 4000) ax = a.x(); // after the first impact
            }
            assertTrue(Math.abs(a.x() - ax) < 0.2, model + ": the robot that stands still was pushed " + (ax - a.x()) + " m");
            assertTrue(b.x() - a.x() < 0.25, model + ": the pusher must not get through");
        }
    }

    @Test
    void holdingForceExceedsPushingForce() {
        RobotSpecs s = new RobotSpecs();
        assertTrue(s.erWheelAccelHold > s.erWheelAccelSpeedup);
        assertTrue(s.erWheelAccelHold * Math.PI * s.erWheelDiameter <= s.wheelMuTraction * 9.81 + 1e-9, "holding is bounded by the wheel grip");
        assertTrue(s.wheelPushRatio < 1.0);
    }
}
