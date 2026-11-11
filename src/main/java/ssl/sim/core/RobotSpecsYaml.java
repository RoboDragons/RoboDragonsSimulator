package ssl.sim.core;

import ssl.sim.robot.RobotSpecs;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * YAML representation of {@link RobotSpecs}: writes a fully commented default file and reads it back (SnakeYAML).
 * Every parameter is preceded by a comment with its physical meaning and unit; the table below must contain every
 * public instance field of RobotSpecs (checked by a test).
 */
public final class RobotSpecsYaml {
    private RobotSpecsYaml() {}

    /** Fields that are not written: rotationError comes from the realism preset. */
    private static final java.util.Set<String> EXCLUDED = java.util.Set.of("rotationError");

    /** Plain angles (not rates) are stored in radians internally but written in degrees (factor rad -> deg). */
    private static final double R2D = 180.0 / Math.PI;
    private static final Map<String, Double> DEG = Map.of("wheelAngles", R2D);

    /** Field name -> "description [unit]". */
    static final Map<String, String> DOC = new LinkedHashMap<>();
    static {
        DOC.put("radius", "Robot body radius [m] (rule: diameter <= 180 mm)");
        DOC.put("height", "Robot height [m] (rule: <= 150 mm)");
        DOC.put("mass", "Robot mass [kg]");
        DOC.put("inertia", "Yaw moment of inertia [kg m^2]; 0 = derived from mass and radius (0.45 m r^2)");
        DOC.put("centerToDribbler", "Distance from the robot centre to the flat front face of the hull [m]");
        DOC.put("maxLinearKickSpeed", "Maximum straight kick ball speed [m/s] (rule: 6.5)");
        DOC.put("maxChipKickSpeed", "Maximum chip kick ball speed [m/s]");
        DOC.put("accSpeedupAbsoluteMax", "Maximum translational acceleration when speeding up [m/s^2]");
        DOC.put("accSpeedupAngularMax", "Maximum angular acceleration when speeding up [rad/s^2]");
        DOC.put("accBrakeAbsoluteMax", "Maximum translational deceleration when braking [m/s^2]");
        DOC.put("accBrakeAngularMax", "Maximum angular deceleration when braking [rad/s^2]");
        DOC.put("velAbsoluteMax", "Maximum translational speed [m/s]");
        DOC.put("velAngularMax", "Maximum yaw rate [rad/s]");
        DOC.put("wheelAngles", "Wheel positions, counter-clockwise from the robot front: front right, back right, back left, front left [deg]");
        DOC.put("wheelMountRadius", "Distance of the wheel contact points from the robot centre [m]");
        DOC.put("wheelRadius", "Wheel radius [m]");
        DOC.put("gearRatio", "Motor-to-wheel gear ratio [-]");
        DOC.put("driveModel", "Drive model: ER_FORCE (body-level velocity control like the ER-Force simulator) or MOTOR (wheel-level DC motor model) [-]");
        DOC.put("erVelocityTimeConstant", "ER_FORCE: time constant of the velocity control (5 ms / ln 2) [s]");
        DOC.put("erWheelAccelSpeedup", "ER_FORCE: wheel acceleration limit when speeding up [rev/s^2]");
        DOC.put("erWheelAccelBrake", "ER_FORCE: wheel acceleration limit when braking [rev/s^2]");
        DOC.put("erWheelDiameter", "ER_FORCE: wheel diameter used to convert wheel revolutions to body speed [m]");
        DOC.put("commandTimeout", "A command expires after this time without a new one; the robot then coasts [s]");
        DOC.put("erLinearDamping", "ER_FORCE: translational damping of a coasting robot [1/s]");
        DOC.put("erAngularDamping", "ER_FORCE: rotational damping of a coasting robot [1/s]");
        DOC.put("ballCoverage", "Fraction of the ball diameter inside the hull seen from above when held (rule: <= 0.2) [-]");
        DOC.put("rollerRadius", "Dribbler roller radius [m]");
        DOC.put("rollerHalfLength", "Dribbler roller half length along the robot y axis [m]");
        DOC.put("rollerContactAngleDeg", "Angle of the roller-ball contact normal above the horizontal [deg]");
        DOC.put("chipperEnabled", "Chipper plate present (three point ball hold) [-]");
        DOC.put("chipperContactAngleDeg", "Angle of the chipper plate-ball contact normal below the horizontal [deg]");
        DOC.put("chipperPlateLength", "Length of the chipper plate [m]");
        DOC.put("chipperTipExtension", "Extension of the chipper plate tip beyond the ball contact point [m]");
        DOC.put("notchHalfWidth", "Half width of the dribbler notch in the hull [m]");
        DOC.put("sideCatchHalfWidth", "Half distance between the side catches (hull parts beside the dribbler notch that touch the ball laterally) [m]; >= radius = no catches");
        DOC.put("rollerDamperEnabled", "Roller can retract rearwards against a spring and damper [-]");
        DOC.put("rollerStroke", "Roller damper stroke [m]");
        DOC.put("rollerSpringK", "Roller damper spring stiffness [N/m]");
        DOC.put("rollerPreload", "Roller damper spring preload force [N]");
        DOC.put("rollerDampingC", "Roller damper damping coefficient [N s/m]");
        DOC.put("rollerSlideMass", "Mass of the sliding roller assembly [kg]");
        DOC.put("kickerStroke", "Kicker plate stroke [m]");
        DOC.put("kickReach", "Largest gap between the kicker plate and the ball for which a kick still reaches it; the stroke is extended over the gap [m]");
        DOC.put("dribblerMaterial", "Ball contact of the dribbler roller rubber: stiffness [N/m], dampingRatio [-], friction coefficient mu [-]");
        DOC.put("chipperMaterial", "Ball contact of the chipper plate: stiffness [N/m], dampingRatio [-], friction coefficient mu [-]");
        DOC.put("kickerMaterial", "Ball contact of the metal kicker plate: stiffness [N/m], dampingRatio [-], friction coefficient mu [-]");
        DOC.put("bodyMaterial", "Ball contact of the robot hull: stiffness [N/m], dampingRatio [-], friction coefficient mu [-]");
        DOC.put("sideCatchMaterial", "Ball contact of the side catches beside the dribbler notch: stiffness [N/m], dampingRatio [-], friction coefficient mu [-]");
        DOC.put("kickerPlateThickness", "Kicker plate thickness [m]");
        DOC.put("kickerPlateHeight", "Kicker plate height [m]");
    }

    /** Complete default file with a comment above every parameter. */
    public static String defaultYaml() {
        return toYaml(new RobotSpecs());
    }

    public static String toYaml(RobotSpecs specs) {
        return ParamsYaml.toYaml(specs, DOC, EXCLUDED,
                "# Robot parameters of the simulator (loaded after config.json; entries may be deleted to use the defaults).\n"
                + "# Units are SI unless stated; plain angles in degrees (converted to radians when loaded), angular rates and accelerations in rad/s and rad/s^2.\n", DEG);
    }

    /** Parses YAML and applies the entries present on top of {@code target}; unknown keys are an error. */
    public static void apply(String yaml, RobotSpecs target) {
        ParamsYaml.apply(yaml, target, RobotSpecs.class, "robot specs", DEG);
    }
}
