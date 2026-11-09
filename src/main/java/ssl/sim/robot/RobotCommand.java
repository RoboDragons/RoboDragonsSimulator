package ssl.sim.robot;

/** Command for one robot, mirroring the SSL simulation protocol's RobotCommand. */
public record RobotCommand(Move move, double kickSpeed, double kickAngleDeg, double dribblerRpm) {

    public sealed interface Move permits WheelVelocity, LocalVelocity, GlobalVelocity {}

    /** Wheel surface velocities [m/s]. */
    public record WheelVelocity(double frontRight, double backRight, double backLeft, double frontLeft) implements Move {}

    /** Forward / left [m/s], angular CCW [rad/s]. */
    public record LocalVelocity(double forward, double left, double angular) implements Move {}

    /** Field-fixed velocity [m/s] and angular CCW [rad/s]. */
    public record GlobalVelocity(double x, double y, double angular) implements Move {}

    public static RobotCommand stop() {
        return new RobotCommand(new LocalVelocity(0, 0, 0), 0, 0, 0);
    }
}
