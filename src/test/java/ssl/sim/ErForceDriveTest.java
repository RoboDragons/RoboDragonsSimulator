package ssl.sim;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.jupiter.api.Test;
import ssl.sim.core.FieldGeometry;
import ssl.sim.core.World;
import ssl.sim.robot.*;

import static org.junit.jupiter.api.Assertions.*;

/** The default drive model reproduces the response of the ER-Force simulator-cli (SimRobot). */
class ErForceDriveTest {
    private static Robot robot(World w) {
        w.ball().teleport(new Vector3D(-4, 3, 0.0215), Vector3D.ZERO, false);
        return w.addRobot(Robot.Team.BLUE, 0, new RobotSpecs(), 0, 0, 0);
    }

    /** Clients re-send their command every cycle (it expires after 0.1 s). */
    private static void drive(World w, Robot r, RobotCommand c, double seconds) {
        for (double t = 0; t < seconds - 1e-9; t += 0.01) {
            r.applyCommand(c);
            w.advance(0.01);
        }
    }

    private static World world() { return new World(new FieldGeometry(FieldGeometry.Division.A)); }

    private static double accel(double f, double l, double a, boolean angular) {
        World w = world();
        Robot r = robot(w);
        RobotCommand c = new RobotCommand(new RobotCommand.LocalVelocity(f, l, a), 0, 0, 0);
        drive(w, r, c, 0.1);
        double v1 = angular ? r.angularVelocity() : r.velocity().getNorm();
        drive(w, r, c, 0.1);
        double v2 = angular ? r.angularVelocity() : r.velocity().getNorm();
        return (v2 - v1) / 0.1;
    }

    @Test
    void accelerationLimitsMatchWheelLimits() {
        // 35 rev/s^2 wheel limit with the 2014 coupling (pseudo-inverse of the clamped wheel accelerations, as in
        // SimRobot::limitAcceleration): 35 (Yf+Yr)/(Yf^2+Yr^2) = 7.6 m/s^2 forward, 35 (Xf+Xr)/(Xf^2+Xr^2) = 6.5 m/s^2 sideways
        assertEquals(7.64, accel(3.5, 0, 0, false), 0.35);
        assertEquals(6.46, accel(0, 3.5, 0, false), 0.35);
        assertEquals(70.7, accel(0, 0, 20, true), 4.0);
    }

    @Test
    void followsSmallVelocityStepsAlmostIdeally() {
        World w = world();
        Robot r = robot(w);
        drive(w, r, new RobotCommand(new RobotCommand.LocalVelocity(0.3, 0, 0), 0, 0, 0), 0.1); // acceleration-limited ramp (~40 ms) plus a few 7 ms time constants
        assertEquals(0.3, r.velocity().getX(), 0.01);
    }

    @Test
    void commandExpiresAndRobotCoastsWithDamping() {
        World w = world();
        Robot r = robot(w);
        drive(w, r, new RobotCommand(new RobotCommand.LocalVelocity(2.0, 0, 0), 0, 0, 0), 1.0);
        assertEquals(2.0, r.velocity().getX(), 0.05);
        w.advance(0.1 + 0.5); // command timeout 0.1 s, then 0.5 s of damping with rate 1.2/s
        double expected = 2.0 * Math.exp(-1.204 * 0.5);
        assertEquals(expected, r.velocity().getX(), 0.15);
    }
}
