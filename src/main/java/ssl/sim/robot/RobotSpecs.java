package ssl.sim.robot;

import ssl.sim.ball.BallContacts;

/**
 * Static robot parameters. Defaults follow the RoboCup SSL rules: the robot must fit in a cylinder
 * of 180 mm diameter and 150 mm height; kick speed is limited to 6.5 m/s.
 * All angles are stored counter-clockwise seen from above, measured from the robot's front (+x).
 */
public class RobotSpecs {
    public double radius = 0.09;
    public double height = 0.15;
    public double mass = 2.5;
    /** Yaw inertia [kg m^2]; 0 = derive from mass and radius (0.45 m r^2). */
    public double inertia = 0;
    /** Distance from robot centre to the flat front face of the convex hull. */
    public double centerToDribbler = 0.075;
    public double maxLinearKickSpeed = 6.5;
    public double maxChipKickSpeed = 6.5;

    // movement limits
    public double accSpeedupAbsoluteMax = 5.0;
    public double accSpeedupAngularMax = 50.0;
    public double accBrakeAbsoluteMax = 5.0;
    public double accBrakeAngularMax = 50.0;
    public double velAbsoluteMax = 4.0;
    public double velAngularMax = 10.0;

    /** Wheel positions as CCW angles [rad]: front right, back right, back left, front left. */
    public double[] wheelAngles = {Math.toRadians(-60), Math.toRadians(-135), Math.toRadians(135), Math.toRadians(60)};
    public double wheelMountRadius = 0.082;
    public double wheelRadius = 0.0295;
    public double gearRatio = 3.33;

    // ---- drive model --------------------------------------------------------------------------------
    /**
     * "ER_FORCE": body-level velocity control like the ER-Force simulator-cli (SimRobot): the body acceleration is
     * (v_cmd - v) / erVelocityTimeConstant, limited through the wheel coupling of the 2014 robot generation, applied as a
     * force/torque on the body; the command expires after commandTimeout and the robot then coasts with damping.
     * "MOTOR": wheel-level model (DC motor, gearbox, PI wheel speed loop, traction) with command acceleration limits.
     */
    public String driveModel = "ER_FORCE";
    /** ER-Force corrects half of the velocity error per 5 ms sub-step: tau = 5 ms / ln 2. */
    public double erVelocityTimeConstant = 0.005 / Math.log(2);
    /** Wheel acceleration limits [rev/s^2] of the ER-Force robot generations (simulation_limits). */
    public double erWheelAccelSpeedup = 35, erWheelAccelBrake = 30;
    /** Diameter used by ER-Force to convert wheel revolutions to body speed. */
    public double erWheelDiameter = 0.045;
    /** Commands expire after this time without a new one (robot coasts). */
    public double commandTimeout = 0.1;
    /** Damping of a coasting robot [1/s], bullet damping 0.7 / 0.8 of the ER-Force simulator. */
    public double erLinearDamping = -Math.log(1 - 0.7), erAngularDamping = -Math.log(1 - 0.8);
    /** ER-Force "robot_rotation_error": yaw acceleration added per forward acceleration. */
    public double rotationError = 0;

    // ---- ball handling geometry (robot frame: x forward, z up) -------------------------------------
    // The ball is held with its centre at holdX(): its rear is ballCoverage * diameter inside the convex hull
    // (SSL rule: at most 20 % of the ball may be covered when seen from above). Roller and chipper plate are
    // placed so that they touch the held ball at the given contact angles:
    //   roller  : behind/above the ball centre, normal pointing forward-down at rollerContactAngleDeg above horizontal
    //   chipper : behind/below the ball centre, normal pointing forward-up at chipperContactAngleDeg below horizontal
    // The roller is therefore always above the kick devices (chipper plate and straight kicker plate).
    public static final double BALL_RADIUS = 0.0215;
    public double ballCoverage = 0.20;
    public double rollerRadius = 0.010;
    public double rollerHalfLength = 0.030;
    public double rollerContactAngleDeg = 30;
    public boolean chipperEnabled = true;
    public double chipperContactAngleDeg = 40;
    public double chipperPlateLength = 0.012;
    public double chipperTipExtension = 0.004;
    public double notchHalfWidth = 0.0325;
    // optional front-back damper of the roller (spring + damper + stroke); the roller can retract rearwards
    public boolean rollerDamperEnabled = false;
    public double rollerStroke = 0.003;
    public double rollerSpringK = 3000;
    public double rollerPreload = 1.0;
    public double rollerDampingC = 20;
    public double rollerSlideMass = 0.03;
    /** Plate stroke [m] of the kicker. */
    public double kickerStroke = 0.02;

    // ---- ball contact materials (stiffness [N/m], dampingRatio, friction coefficient mu) -------------
    /** Rubber of the dribbler roller (high friction). */
    public BallContacts.Material dribblerMaterial = new BallContacts.Material(3.0e4, 0.8, 2.0);
    /** Chipper plate (passive support of the held ball). */
    public BallContacts.Material chipperMaterial = new BallContacts.Material(5.0e4, 0.5, 0.3);
    /** Metal kicker plate: stiff, strongly damped (little rebound). */
    public BallContacts.Material kickerMaterial = new BallContacts.Material(1.0e5, 0.7, 0.4);
    /** Robot body (hull) against the ball. */
    public BallContacts.Material bodyMaterial = new BallContacts.Material(5.0e4, 0.3, 0.5);
    public double kickerPlateThickness = 0.006;
    public double kickerPlateHeight = 0.008;

    /** Ball centre x when held (top view: ballCoverage * diameter of the ball lies inside the convex hull). */
    public double holdX() { return centerToDribbler + BALL_RADIUS * (1 - 2 * nominalCoverage()); }

    /**
     * Coverage with the roller at its nominal (extended) position. With a damper the nominal position is chosen so
     * that even a fully retracted roller (ball pushed back by the stroke) covers at most {@code ballCoverage}.
     */
    public double nominalCoverage() {
        return ballCoverage - (rollerDamperEnabled ? rollerStroke / (2 * BALL_RADIUS) : 0);
    }

    /** Largest coverage the ball can reach (roller fully retracted). */
    public double maxTopViewCoverage() {
        return topViewCoverage() + (rollerDamperEnabled ? rollerStroke / (2 * BALL_RADIUS) : 0);
    }

    public double rollerAxisX() { return holdX() - (BALL_RADIUS + rollerRadius) * Math.cos(Math.toRadians(rollerContactAngleDeg)); }

    public double rollerAxisZ() { return BALL_RADIUS + (BALL_RADIUS + rollerRadius) * Math.sin(Math.toRadians(rollerContactAngleDeg)); }

    /** Ball centre x when the roller is fully retracted (= holdX() without a damper). */
    public double compressedHoldX() { return centerToDribbler + BALL_RADIUS * (1 - 2 * ballCoverage); }

    /** Point where the chipper plate surface touches the ball at the compressed hold position (x, z). */
    public double[] chipperContact() {
        double d = Math.toRadians(chipperContactAngleDeg);
        return new double[]{compressedHoldX() - BALL_RADIUS * Math.cos(d), BALL_RADIUS * (1 - Math.sin(d))};
    }

    /** Front (low) end of the chipper plate (x, z): slightly beyond the contact point, along the plate surface. */
    public double[] chipperTip() {
        double d = Math.toRadians(chipperContactAngleDeg);
        double[] c = chipperContact();
        return new double[]{c[0] + chipperTipExtension * Math.sin(d), Math.max(0.001, c[1] - chipperTipExtension * Math.cos(d))};
    }

    /** Rear (high) end of the chipper plate (x, z). */
    public double[] chipperTop() {
        double d = Math.toRadians(chipperContactAngleDeg);
        double[] c = chipperContact();
        return new double[]{c[0] - chipperPlateLength * Math.sin(d), c[1] + chipperPlateLength * Math.cos(d)};
    }

    /** Front face of the (retracted) straight kicker plate: just behind the ball's rear tip at centre height. */
    public double kickerFrontX() { return compressedHoldX() - BALL_RADIUS - 0.001; }

    public double kickerCenterZ() { return BALL_RADIUS; }

    /** x of the floor of the dribbler notch, behind the roller. */
    public double notchFloorX() { return rollerAxisX() - rollerRadius - 0.005; }

    /** Fraction of the ball diameter inside the convex hull (seen from above) when the ball is held. */
    public double topViewCoverage() { return (centerToDribbler - (holdX() - BALL_RADIUS)) / (2 * BALL_RADIUS); }

    /** Frontmost x of any device (roller, chipper plate, kicker plate); must not exceed the hull face. */
    public double frontmostDeviceX() {
        return Math.max(rollerAxisX() + rollerRadius, Math.max(chipperEnabled ? chipperTip()[0] : -1, kickerFrontX()));
    }

    /** Human-readable layout problems (empty if the layout is consistent). */
    public java.util.List<String> validateLayout() {
        java.util.List<String> p = new java.util.ArrayList<>();
        if (maxTopViewCoverage() > 0.2 + 1e-9) p.add("ball covered more than 20 % from above");
        if (frontmostDeviceX() > centerToDribbler + 1e-9) p.add("a device sticks out of the convex hull");
        if (chipperEnabled && rollerAxisZ() - rollerRadius <= chipperTop()[1])
            p.add("roller is not above the chipper plate");
        if (rollerAxisZ() - rollerRadius <= kickerCenterZ() + kickerPlateHeight / 2)
            p.add("roller is not above the kicker plate");
        return p;
    }

    public double effectiveInertia() { return inertia > 0 ? inertia : 0.45 * mass * radius * radius; }

    public RobotSpecs copy() {
        RobotSpecs s = new RobotSpecs();
        s.radius = radius;
        s.height = height;
        s.mass = mass;
        s.inertia = inertia;
        s.centerToDribbler = centerToDribbler;
        s.maxLinearKickSpeed = maxLinearKickSpeed;
        s.maxChipKickSpeed = maxChipKickSpeed;
        s.accSpeedupAbsoluteMax = accSpeedupAbsoluteMax;
        s.accSpeedupAngularMax = accSpeedupAngularMax;
        s.accBrakeAbsoluteMax = accBrakeAbsoluteMax;
        s.accBrakeAngularMax = accBrakeAngularMax;
        s.driveModel = driveModel;
        s.erVelocityTimeConstant = erVelocityTimeConstant;
        s.erWheelAccelSpeedup = erWheelAccelSpeedup;
        s.erWheelAccelBrake = erWheelAccelBrake;
        s.erWheelDiameter = erWheelDiameter;
        s.commandTimeout = commandTimeout;
        s.erLinearDamping = erLinearDamping;
        s.erAngularDamping = erAngularDamping;
        s.rotationError = rotationError;
        s.velAbsoluteMax = velAbsoluteMax;
        s.velAngularMax = velAngularMax;
        s.wheelAngles = wheelAngles.clone();
        s.wheelMountRadius = wheelMountRadius;
        s.wheelRadius = wheelRadius;
        s.gearRatio = gearRatio;
        s.rollerRadius = rollerRadius;
        s.ballCoverage = ballCoverage;
        s.rollerHalfLength = rollerHalfLength;
        s.rollerContactAngleDeg = rollerContactAngleDeg;
        s.chipperEnabled = chipperEnabled;
        s.chipperContactAngleDeg = chipperContactAngleDeg;
        s.chipperPlateLength = chipperPlateLength;
        s.chipperTipExtension = chipperTipExtension;
        s.notchHalfWidth = notchHalfWidth;
        s.rollerDamperEnabled = rollerDamperEnabled;
        s.rollerStroke = rollerStroke;
        s.rollerSpringK = rollerSpringK;
        s.rollerPreload = rollerPreload;
        s.rollerDampingC = rollerDampingC;
        s.rollerSlideMass = rollerSlideMass;
        s.kickerStroke = kickerStroke;
        s.dribblerMaterial = dribblerMaterial;
        s.chipperMaterial = chipperMaterial;
        s.kickerMaterial = kickerMaterial;
        s.bodyMaterial = bodyMaterial;
        s.kickerPlateThickness = kickerPlateThickness;
        s.kickerPlateHeight = kickerPlateHeight;
        return s;
    }
}
