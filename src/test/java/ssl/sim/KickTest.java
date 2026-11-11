package ssl.sim;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.jupiter.api.Test;
import ssl.sim.core.FieldGeometry;
import ssl.sim.core.World;
import ssl.sim.robot.*;

import static org.junit.jupiter.api.Assertions.*;

class KickTest {
    /** Holds the ball with the spinning dribbler, then kicks. Returns the world after {@code after} seconds. */
    private static World kick(double friction, double rpm, double speed, double angle, double after, double[] out) {
        World w = new World(new FieldGeometry(FieldGeometry.Division.A));
        RobotSpecs s = new RobotSpecs();
        s.kickerMaterial = new ssl.sim.ball.BallContacts.Material(1.0e5, 0.7, friction);
        Robot r = w.addRobot(Robot.Team.BLUE, 0, s, -3, 0, 0);
        w.ball().teleport(new Vector3D(-2.65, 0, 0.0215), Vector3D.ZERO, false);
        for (int i = 0; i < 250; i++) { // approach at low roller speed, then raise it slowly (a sudden spin-up of the resting ball throws it off the roller)
            double roller = rpm == 0 || i < 100 ? Math.min(rpm, 2000) : i < 200 ? 2000 + (rpm - 2000) * (i - 99) / 100.0 : rpm;
            r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(i < 100 ? 0.4 : 0, 0, 0), 0, 0, Math.max(roller, 0)));
            w.advance(0.01);
        }
        if (rpm == 0) { // no dribbler: place the ball at the hold position (it would be pushed away otherwise)
            var hold = r.toWorld(s.holdX(), 0);
            w.ball().teleport(new Vector3D(hold.getX(), hold.getY(), 0.0215), Vector3D.ZERO, false);
        }
        out[0] = w.ball().angularVelocity().getY(); // spin before the kick
        r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(0, 0, 0), speed, angle, rpm));
        double vmax = 0;
        for (int i = 0; i < after / 0.001; i++) {
            r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(0, 0, 0), 0, 0, rpm));
            w.advance(0.001);
            if (i == 30) out[1] = w.ball().velocity().getX();          // speed 30 ms after the kick
            if (i == 30) out[2] = w.ball().angularVelocity().getY();   // spin 30 ms after the kick
            vmax = Math.max(vmax, w.ball().z());
        }
        out[3] = vmax;
        return w;
    }

    @Test
    void kickSpeedMatchesCommand() {
        for (double v : new double[]{2, 4, 6}) {
            double[] o = new double[4];
            kick(0.4, 0, v, 0, 0.05, o);
            System.out.printf("kick %.1f m/s -> ball %.2f m/s%n", v, o[1]);
            assertEquals(v, o[1], 0.12 * v + 0.2, "commanded " + v);
        }
    }

    @Test
    void frictionRemovesBackspinSoTheBallDoesNotComeBack() {
        double[] withF = new double[4], noF = new double[4];
        World a = kick(0.4, 9000, 5.0, 0, 1.0, withF);
        World b = kick(0.0, 9000, 5.0, 0, 1.0, noF);
        System.out.printf("backspin before %.0f rad/s | with friction: v30=%.2f spin30=%.0f final v=%.2f x=%.2f | without: v30=%.2f spin30=%.0f final v=%.2f x=%.2f%n",
                withF[0], withF[1], withF[2], a.ball().velocity().getX(), a.ball().x(), noF[1], noF[2], b.ball().velocity().getX(), b.ball().x());
        assertTrue(withF[0] < -200, "dribbler should give strong backspin");
        assertTrue(a.ball().velocity().getX() > 1.0, "ball must keep going forward with friction");
        assertTrue(a.ball().x() > -2.65 + 1.0);
        assertTrue(Math.abs(withF[2]) < Math.abs(noF[2]), "plate friction removes spin");
    }

    @Test
    void chipKickLeavesTheGround() {
        double[] o = new double[4];
        kick(0.4, 0, 4.0, 45, 0.5, o);
        System.out.printf("chip 4 m/s at 45 deg: max height %.2f m%n", o[3]);
        assertTrue(o[3] > 0.15);
    }

    /** Ball speed 100 ms after a 4 m/s kick of a resting ball that lies {@code gap} metres in front of the hold position (no dribbler). */
    private static double kickFromGap(double gap, boolean chip) {
        World w = new World(new FieldGeometry(FieldGeometry.Division.A));
        RobotSpecs s = new RobotSpecs();
        Robot r = w.addRobot(Robot.Team.BLUE, 0, s, -3, 0, 0);
        var p = r.toWorld(s.holdX() + gap, 0);
        w.ball().teleport(new Vector3D(p.getX(), p.getY(), 0.0215), Vector3D.ZERO, false);
        r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(0, 0, 0), 4.0, chip ? 45 : 0, 0));
        for (int i = 0; i < 0.1 / 0.001; i++) {
            r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(0, 0, 0), i == 0 ? 4.0 : 0, chip ? 45 : 0, 0));
            w.advance(0.001);
        }
        return w.ball().velocity().getNorm();
    }

    @Test
    void closeButNotTouchingBallIsStillKicked() {
        double touching = kickFromGap(0.0, false); // ~3.6 m/s 100 ms after a 4 m/s kick (ground friction)
        assertEquals(4.0, touching, 0.6);
        for (double gap : new double[]{0.005, 0.009}) { // plate gaps of 6 and 10 mm (a held ball has 1 mm)
            double v = kickFromGap(gap, false);
            System.out.printf("gap %.0f mm: ball speed %.2f m/s (touching: %.2f)%n", gap * 1000, v, touching);
            assertEquals(touching, v, 0.4, "gap " + gap);
        }
        for (double gap : new double[]{0.01}) {
            double v = kickFromGap(gap, true);
            System.out.printf("chip, gap %.0f mm: ball speed %.2f m/s%n", gap * 1000, v);
            assertTrue(v > 2.0, "chip gap " + gap + " speed " + v);
        }
        assertTrue(kickFromGap(0.025, false) < 0.5, "beyond the stroke the ball is not hit");
    }
}
