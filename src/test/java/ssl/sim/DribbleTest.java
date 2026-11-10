package ssl.sim;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.jupiter.api.Test;
import ssl.sim.ball.Ball;
import ssl.sim.core.FieldGeometry;
import ssl.sim.core.World;
import ssl.sim.robot.*;

import static org.junit.jupiter.api.Assertions.*;

class DribbleTest {
    private static World world() { return new World(new FieldGeometry(FieldGeometry.Division.A)); }

    @Test
    void layoutRespectsRules() {
        RobotSpecs s = TestSpecs.motor();
        assertEquals(0.20, s.topViewCoverage(), 1e-9, "ball must be covered 20 % from above");
        assertTrue(s.validateLayout().isEmpty(), s.validateLayout().toString());
        // dribbler above kick devices
        assertTrue(s.rollerAxisZ() - s.rollerRadius > s.chipperTop()[1]);
        assertTrue(s.rollerAxisZ() - s.rollerRadius > s.kickerCenterZ() + s.kickerPlateHeight / 2);
        assertTrue(s.rollerAxisZ() > s.chipperContact()[1]);
        // devices stay inside the convex hull
        assertTrue(s.frontmostDeviceX() <= s.centerToDribbler);
        // the held ball really touches roller and chipper plate (3-point geometry)
        double R = RobotSpecs.BALL_RADIUS;
        double dr = Math.hypot(s.holdX() - s.rollerAxisX(), R - s.rollerAxisZ());
        assertEquals(R + s.rollerRadius, dr, 1e-9);
        double[] c = s.chipperContact();
        assertEquals(R, Math.hypot(s.holdX() - c[0], R - c[1]), 1e-9);
    }

    @Test
    void staticsThreeAndTwoPoint() {
        RobotSpecs s = TestSpecs.motor();
        var both = DribblerStatics.analyse(s, 2.0, 0.3, 0.4, 0.046);
        System.out.println("3pt " + both.threePoint());
        System.out.println("2pt " + both.twoPoint());
        assertTrue(both.threePoint().stable());
        assertTrue(both.twoPoint().stable());
        // the chipper takes load off the ground contact (it pushes the ball up)
        assertTrue(both.threePoint().groundNormal() < both.twoPoint().groundNormal());
        // 3-point: ground friction + chipper friction balance the roller friction (torque balance)
        var t = both.threePoint();
        assertEquals(2.0 * t.rollerNormal(), 0.4 * t.groundNormal() + 0.3 * t.chipperNormal(), 1e-9);
    }

    private static double holdAndMeasure(RobotSpecs s, boolean chipper, double[] out) {
        World w = world();
        Robot r = w.addRobot(Robot.Team.BLUE, 0, s, 0, 0, 0);
        w.ball().teleport(new Vector3D(0.35, 0, 0.0215), Vector3D.ZERO, false);
        r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(0.4, 0, 0), 0, 0, 9000));
        w.advance(1.0);
        r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(0, 0, 0), 0, 0, 9000));
        w.advance(1.5);
        double nr = 0, nc = 0;
        int n = 0;
        for (int i = 0; i < 2000; i++) {
            w.step(World.PHYSICS_DT);
            nr += r.dribbler().contactNormalForce();
            nc += r.chipper().contactNormalForce();
            n++;
        }
        out[0] = nr / n;
        out[1] = nc / n;
        out[2] = r.dribbler().hasBallContact() ? 1 : 0;
        out[3] = r.chipper().hasBallContact() ? 1 : 0;
        return w.ball().position2D().distance(r.toWorld(s.holdX(), 0));
    }

    @Test
    void threePointHoldMatchesStatics() {
        RobotSpecs s = TestSpecs.motor();
        double[] o = new double[4];
        double dist = holdAndMeasure(s, true, o);
        var st = DribblerStatics.threePoint(s, 2.0, 0.3, 0.4, 0.046);
        System.out.printf("3pt sim: dist=%.4f Nr=%.3f (static %.3f) Nc=%.3f (static %.3f) contacts=%s%n", dist, o[0], st.rollerNormal(), o[1], st.chipperNormal(), java.util.Arrays.toString(o));
        assertTrue(dist < 0.004, "hold position drift " + dist);
        assertEquals(1, o[2]);
        assertEquals(1, o[3]);
        assertEquals(st.rollerNormal(), o[0], 0.6 * st.rollerNormal());
        assertEquals(st.chipperNormal(), o[1], 0.5 * st.chipperNormal());
    }

    @Test
    void twoPointHoldMatchesStatics() {
        RobotSpecs s = TestSpecs.motor();
        s.chipperEnabled = false;
        double[] o = new double[4];
        double dist = holdAndMeasure(s, false, o);
        var st = DribblerStatics.twoPoint(s, 2.0, 0.4, 0.046);
        System.out.printf("2pt sim: dist=%.4f Nr=%.3f (static %.3f) contacts=%s%n", dist, o[0], st.rollerNormal(), java.util.Arrays.toString(o));
        assertTrue(dist < 0.004, "hold position drift " + dist);
        assertEquals(1, o[2]);
        assertEquals(0, o[3]);
        assertEquals(st.rollerNormal(), o[0], 0.5 * st.rollerNormal());
    }

    @Test
    void ballRidesWithRobotBackwardsBothModes() {
        for (boolean chipper : new boolean[]{true, false}) {
            RobotSpecs s = TestSpecs.motor();
            s.chipperEnabled = chipper;
            s.accSpeedupAbsoluteMax = 1.0;
            s.accBrakeAbsoluteMax = 1.0;
            World w = world();
            Robot r = w.addRobot(Robot.Team.BLUE, 0, s, 0, 0, 0);
            w.ball().teleport(new Vector3D(0.35, 0, 0.0215), Vector3D.ZERO, false);
            r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(0.4, 0, 0), 0, 0, 9000));
            w.advance(1.0);
            r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(0, 0, 0), 0, 0, 9000));
            w.advance(1.0);
            r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(-0.5, 0, 0.0), 0, 0, 9000));
            w.advance(1.5);
            double d = w.ball().position2D().distance(r.toWorld(s.holdX(), 0));
            assertTrue(d < 0.01, "chipper=" + chipper + " drift " + d);
        }
    }

    /** A ball pressed against a wall can be peeled off by a spinning dribbler (needs low ball-wall friction). */
    private static double peelFromWall(double muWall) {
        Ball.Params bp = new Ball.Params();
        bp.muWall = muWall;
        World w = new World(new FieldGeometry(FieldGeometry.Division.A), bp);
        double wallX = w.field().halfLength() + w.field().boundaryGoalLine;
        RobotSpecs s = TestSpecs.motor();
        s.accSpeedupAbsoluteMax = 1.0;
        s.accBrakeAbsoluteMax = 1.0;
        Robot r = w.addRobot(Robot.Team.BLUE, 0, s, wallX - 0.6, 2.0, 0);
        w.ball().teleport(new Vector3D(wallX - 0.0216, 2.0, 0.0215), Vector3D.ZERO, false);
        r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(0.4, 0, 0), 0, 0, 9000));
        for (int i = 0; i < 4000 && r.x() < wallX - s.holdX() - 0.0216 + 0.002; i++) w.step(World.PHYSICS_DT);
        r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(0, 0, 0), 0, 0, 9000));
        w.advance(0.8);
        r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(-0.5, 0, 0), 0, 0, 9000));
        w.advance(2.0);
        return wallX - w.ball().x();
    }

    @Test
    void dribblerPeelsBallOffWall() {
        double d = peelFromWall(0.05);
        System.out.println("peel distance (mu_wall 0.05): " + d + ", with mu_wall 0.3: " + peelFromWall(0.3));
        assertTrue(d > 0.3, "ball should follow the robot away from the wall, distance " + d);
    }

    @Test
    void rollerDamperKeepsTwentyPercentRuleAndHolds() {
        RobotSpecs s = TestSpecs.motor();
        s.rollerDamperEnabled = true;
        assertTrue(s.validateLayout().isEmpty(), s.validateLayout().toString());
        assertEquals(0.20, s.maxTopViewCoverage(), 1e-9);
        assertTrue(s.topViewCoverage() < 0.2);
        double[] o = new double[4];
        double dist = holdAndMeasure(s, true, o);
        assertTrue(dist < 0.006, "hold drift with damper " + dist);
        assertEquals(1, o[2]);
    }
}
