package ssl.sim;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.jupiter.api.Test;
import ssl.sim.ball.Ball;
import ssl.sim.core.FieldGeometry;
import ssl.sim.core.World;
import ssl.sim.robot.Robot;
import ssl.sim.robot.RobotCommand;
import ssl.sim.robot.RobotSpecs;

import static org.junit.jupiter.api.Assertions.*;

class PhysicsTest {
    private static World newWorld() {
        return new World(new FieldGeometry(FieldGeometry.Division.A));
    }

    @Test
    void ballSlidesThenRolls() {
        World w = newWorld();
        w.ball().teleport(new Vector3D(0, 0, 0.0215), new Vector3D(5, 0, 0), false);
        w.advance(0.2);
        // sliding with mu*g = 3.9 m/s^2: speed drops fast
        assertTrue(w.ball().velocity().getX() < 4.5);
        w.advance(0.5);
        double v = w.ball().velocity().getX();
        double roll = w.ball().angularVelocity().getY() * 0.0215;
        assertEquals(v, roll, 0.02, "ball should roll without slip after sliding phase");
        double v1 = v;
        w.advance(1.0);
        assertEquals(0.35, v1 - w.ball().velocity().getX(), 0.05, "rolling deceleration");
    }

    @Test
    void backspinBallReturns() {
        World w = newWorld();
        w.ball().teleport(new Vector3D(0, 0, 0.0215), new Vector3D(1.0, 0, 0), false);
        w.ball().setAngularVelocity(new Vector3D(0, -300, 0)); // strong backspin
        w.advance(1.0);
        assertTrue(w.ball().velocity().getX() < 0, "backspin should reverse the ball");
    }

    @Test
    void chipBounces() {
        World w = newWorld();
        w.ball().teleport(new Vector3D(0, 0, 0.0215), new Vector3D(3, 0, 3), false);
        double maxZ = 0;
        for (int i = 0; i < 6000; i++) {
            w.step(World.PHYSICS_DT);
            maxZ = Math.max(maxZ, w.ball().z());
        }
        assertTrue(maxZ > 0.3);
        assertTrue(w.ball().z() < 0.1);
    }

    @Test
    void robotDrivesAtCommandedVelocity() {
        World w = newWorld();
        w.ball().teleport(new Vector3D(-3, 3, 0.0215), Vector3D.ZERO, false);
        Robot r = w.addRobot(Robot.Team.BLUE, 0, TestSpecs.motor(), 0, 0, 0);
        r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(2.0, 0, 0), 0, 0, 0));
        w.advance(2.0);
        assertEquals(2.0, r.velocity().getX(), 0.15);
        assertEquals(0.0, r.velocity().getY(), 0.1);
        r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(0, 0, 3.0), 0, 0, 0));
        w.advance(2.0);
        assertEquals(3.0, r.angularVelocity(), 0.4);
    }

    @Test
    void kickerHitsBall() {
        World w = newWorld();
        Robot r = w.addRobot(Robot.Team.YELLOW, 1, TestSpecs.motor(), 0, 0, 0);
        w.ball().teleport(new Vector3D(0.075 + 0.0215 + 0.002, 0, 0.0215), Vector3D.ZERO, false);
        r.applyCommand(new RobotCommand(null, 5.0, 0, 0));
        w.advance(0.05);
        assertEquals(5.0, w.ball().velocity().getX(), 0.5);
    }

    @Test
    void dribblerHoldsBallWhileDrivingBackwards() {
        World w = newWorld();
        RobotSpecs s = TestSpecs.motor();
        s.accSpeedupAbsoluteMax = 1.0;
        s.accBrakeAbsoluteMax = 1.0;
        Robot r = w.addRobot(Robot.Team.BLUE, 0, s, 0, 0, 0);
        double holdX = s.centerToDribbler + 0.0129;
        w.ball().teleport(new Vector3D(0.35, 0, 0.0215), Vector3D.ZERO, false);
        r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(0.4, 0, 0), 0, 0, 9000));
        w.advance(1.0);
        r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(0, 0, 0), 0, 0, 9000));
        w.advance(1.0);
        assertTrue(r.dribbler().hasBallContact(), "ball should touch the roller");
        r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(-0.5, 0, 0), 0, 0, 9000));
        w.advance(1.5);
        double dist = w.ball().position2D().distance(r.toWorld(holdX, 0));
        assertTrue(dist < 0.03, "ball drifted away from dribbler: " + dist);
        // and without the dribbler the ball is left behind
        World w2 = newWorld();
        Robot r2 = w2.addRobot(Robot.Team.BLUE, 0, s, 0, 0, 0);
        w2.ball().teleport(new Vector3D(holdX + 0.001, 0, 0.0215), Vector3D.ZERO, false);
        r2.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(-0.5, 0, 0), 0, 0, 0));
        w2.advance(1.5);
        double dist2 = w2.ball().position2D().distance(r2.toWorld(s.centerToDribbler + 0.0129, 0));
        assertTrue(dist2 > 0.3, "without dribbler ball should be left: " + dist2);
    }

    @Test
    void ballStaysInsideWalls() {
        World w = newWorld();
        w.ball().teleport(new Vector3D(0, 0, 0.0215), new Vector3D(6.5, 2, 0), true);
        w.advance(8.0);
        FieldGeometry f = w.field();
        assertTrue(Math.abs(w.ball().x()) < f.halfLength() + f.boundaryGoalLine && Math.abs(w.ball().y()) < f.halfWidth() + f.boundaryTouchLine);
    }

    @Test
    void robotsDoNotTunnelThroughWalls() {
        double[][] dirs = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {1, 1, 0.5}, {-1, 1, -0.5}, {1, 0, 4}};
        for (double[] d : dirs) {
            World w = newWorld();
            FieldGeometry f = w.field();
            w.ball().teleport(new Vector3D(0, 0.5, 0.0215), Vector3D.ZERO, false);
            Robot r = w.addRobot(Robot.Team.BLUE, 0, TestSpecs.motor(), 0, 0, 0);
            r.applyCommand(new RobotCommand(new RobotCommand.GlobalVelocity(d[0] * 3.5, d[1] * 3.5, d[2]), 0, 0, 0));
            for (int i = 0; i < 12000; i++) {
                w.step(World.PHYSICS_DT);
                assertTrue(Math.abs(r.x()) < f.halfLength() + f.boundaryGoalLine + 0.02
                                || Math.abs(r.y()) < f.goalWidth / 2 && Math.abs(r.x()) < f.halfLength() + f.goalDepth + 0.02,
                        "robot escaped in x: " + r.position() + " dir " + java.util.Arrays.toString(d));
                assertTrue(Math.abs(r.y()) < f.halfWidth() + f.boundaryTouchLine + 0.02,
                        "robot escaped in y: " + r.position() + " dir " + java.util.Arrays.toString(d));
            }
        }
    }

    @Test
    void robotCollisionsAreStableAndDoNotOverlap() {
        World w = newWorld();
        w.ball().teleport(new Vector3D(0, 3, 0.0215), Vector3D.ZERO, false);
        Robot a = w.addRobot(Robot.Team.BLUE, 0, TestSpecs.motor(), -1.5, 0, 0);
        Robot b = w.addRobot(Robot.Team.YELLOW, 0, TestSpecs.motor(), 1.5, 0.05, Math.PI);
        a.applyCommand(new RobotCommand(new RobotCommand.GlobalVelocity(3.5, 0, 0), 0, 0, 0));
        b.applyCommand(new RobotCommand(new RobotCommand.GlobalVelocity(-3.5, 0, 0), 0, 0, 0));
        double minDist = 10;
        for (int i = 0; i < 12000; i++) {
            w.step(World.PHYSICS_DT);
            minDist = Math.min(minDist, a.position().distance(b.position()));
            assertTrue(a.velocity().getNorm() < 6 && Math.abs(a.angularVelocity()) < 60, "unstable at t=" + w.time());
        }
        assertTrue(minDist > 0.12, "robots overlap too much: " + minDist);
    }

    @Test
    void maxTranslationalAccelerationIsFiveInAnyDirection() {
        for (double ang : new double[]{0, Math.PI / 2, Math.PI / 4, 2.0, Math.PI}) {
            World w = newWorld();
            w.ball().teleport(new Vector3D(-4, 3, 0.0215), Vector3D.ZERO, false);
            Robot r = w.addRobot(Robot.Team.BLUE, 0, TestSpecs.motor(), 0, 0, 0.7);
            double vx = Math.cos(ang) * 3.5, vy = Math.sin(ang) * 3.5;
            r.applyCommand(new RobotCommand(new RobotCommand.GlobalVelocity(vx, vy, 0), 0, 0, 0));
            w.advance(0.3);
            double a = r.velocity().getNorm() / 0.3; // average over the first 0.3 s
            assertEquals(5.0, a, 0.5, "acceleration towards " + ang);
            // braking is limited to 5 m/s^2 as well
            w.advance(1.0);
            r.applyCommand(new RobotCommand(new RobotCommand.GlobalVelocity(0, 0, 0), 0, 0, 0));
            double v0 = r.velocity().getNorm();
            w.advance(0.3);
            assertEquals(5.0, (v0 - r.velocity().getNorm()) / 0.3, 0.7, "braking towards " + ang);
        }
    }
}
