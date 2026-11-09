package ssl.sim;

import ssl.sim.robot.RobotSpecs;

/** Specs of the wheel-level "MOTOR" drive model (commands persist; used by the tests written for that model). */
final class TestSpecs {
    private TestSpecs() {}

    static RobotSpecs motor() {
        RobotSpecs s = new RobotSpecs();
        s.driveModel = "MOTOR";
        return s;
    }
}
