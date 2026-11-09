package ssl.sim.robot;

import ssl.sim.ball.Ball;

/**
 * Four (or N) omni wheel drive train: converts body velocity commands (with acceleration limits)
 * into wheel speed set points via inverse kinematics.
 */
public class OmniDrive implements RobotComponent {
    private final OmniWheel[] wheels;
    private final RobotSpecs specs;
    private RobotCommand.Move move = new RobotCommand.LocalVelocity(0, 0, 0);
    private double cmdVx, cmdVy, cmdW; // current acceleration-limited set point in the body frame
    private final boolean erForce;
    private double lastCommandTime = Double.NEGATIVE_INFINITY;
    private final org.apache.commons.math3.linear.RealMatrix coupling, couplingPinv;

    public OmniDrive(RobotSpecs specs) {
        this.specs = specs;
        this.erForce = "ER_FORCE".equalsIgnoreCase(specs.driveModel);
        double d = specs.erWheelDiameter, k = 1.0 / Math.PI / d;
        double xf = k * Math.cos(Math.toRadians(35)), yf = k * Math.sin(Math.toRadians(35));
        double xr = k * Math.cos(Math.toRadians(45)), yr = k * Math.sin(Math.toRadians(45)), phi = 29.7 / 60;
        // rows: wheel speed [rev/s] for (v_right, v_forward, omega); same structure as ER-Force SimRobot::generateVelocityCoupling
        this.coupling = org.apache.commons.math3.linear.MatrixUtils.createRealMatrix(new double[][]{
                {xf, yf, -phi}, {-xr, yr, -phi}, {-xr, -yr, -phi}, {xf, -yf, -phi}});
        this.couplingPinv = new org.apache.commons.math3.linear.SingularValueDecomposition(coupling).getSolver().getInverse();
        wheels = new OmniWheel[specs.wheelAngles.length];
        for (int i = 0; i < wheels.length; i++) {
            wheels[i] = new OmniWheel(specs.wheelAngles[i], specs.wheelMountRadius, specs.wheelRadius,
                    specs.gearRatio, DCMotor.wheelMotor());
        }
    }

    public OmniWheel[] wheels() { return wheels; }

    public void setMove(RobotCommand.Move move) { setMove(move, 0); }

    /** Sets the movement command; {@code time} is the simulation time it was received (for the command timeout). */
    public void setMove(RobotCommand.Move move, double time) {
        if (move != null) this.move = move;
        else if (erForce) this.move = null; // a command without move part makes the ER-Force robot coast
        lastCommandTime = time;
    }

    /** Current acceleration-limited body-frame velocity set point (forward, left, angular). */
    public double[] setpointLocal() { return new double[]{cmdVx, cmdVy, cmdW}; }

    @Override
    public void update(Robot robot, Ball ball, double dt) {
        if (erForce) {
            updateErForce(robot, dt);
            return;
        }
        double[] target = new double[wheels.length];
        if (move instanceof RobotCommand.WheelVelocity wv && wheels.length == 4) {
            target[0] = wv.frontRight();
            target[1] = wv.backRight();
            target[2] = wv.backLeft();
            target[3] = wv.frontLeft();
            cmdVx = cmdVy = cmdW = 0;
        } else {
            double tvx, tvy, tw;
            if (move instanceof RobotCommand.GlobalVelocity g) {
                double c = Math.cos(robot.heading()), s = Math.sin(robot.heading());
                tvx = c * g.x() + s * g.y();
                tvy = -s * g.x() + c * g.y();
                tw = g.angular();
            } else if (move instanceof RobotCommand.LocalVelocity l) {
                tvx = l.forward();
                tvy = l.left();
                tw = l.angular();
            } else {
                tvx = tvy = tw = 0;
            }
            double speed = Math.hypot(tvx, tvy);
            if (speed > specs.velAbsoluteMax) {
                tvx *= specs.velAbsoluteMax / speed;
                tvy *= specs.velAbsoluteMax / speed;
            }
            tw = Math.max(-specs.velAngularMax, Math.min(specs.velAngularMax, tw));
            limitAcceleration(tvx, tvy, tw, dt);
            for (int i = 0; i < wheels.length; i++) target[i] = wheels[i].surfaceSpeedFor(cmdVx, cmdVy, cmdW);
        }
        for (int i = 0; i < wheels.length; i++) wheels[i].update(robot, target[i], wheels.length, dt);
    }

    private void limitAcceleration(double tvx, double tvy, double tw, double dt) {
        double dvx = tvx - cmdVx, dvy = tvy - cmdVy;
        double dv = Math.hypot(dvx, dvy);
        boolean speeding = Math.hypot(tvx, tvy) > Math.hypot(cmdVx, cmdVy);
        double aMax = (speeding ? specs.accSpeedupAbsoluteMax : specs.accBrakeAbsoluteMax) * dt;
        if (dv > aMax) {
            dvx *= aMax / dv;
            dvy *= aMax / dv;
        }
        cmdVx += dvx;
        cmdVy += dvy;
        double dw = tw - cmdW;
        double wMax = (Math.abs(tw) > Math.abs(cmdW) ? specs.accSpeedupAngularMax : specs.accBrakeAngularMax) * dt;
        cmdW += Math.max(-wMax, Math.min(wMax, dw));
    }

    /** Body-level velocity controller of the ER-Force simulator (see SimRobot::begin). */
    private void updateErForce(Robot robot, double dt) {
        double c = Math.cos(robot.heading()), sn = Math.sin(robot.heading());
        double vx = robot.velocity().getX(), vy = robot.velocity().getY();
        double vF = c * vx + sn * vy, vR = sn * vx - c * vy, w = robot.angularVelocity();
        boolean active = move != null && robot.time() - lastCommandTime <= specs.commandTimeout;
        if (!active) { // standby: the command expired, the robot coasts and is only damped
            cmdVx = cmdVy = cmdW = 0;
            robot.applyForce(new org.apache.commons.math3.geometry.euclidean.twod.Vector2D(
                    -robot.mass() * specs.erLinearDamping * vx, -robot.mass() * specs.erLinearDamping * vy));
            robot.applyTorque(-robot.inertia() * specs.erAngularDamping * w);
            syncWheels(vF, -vR, w);
            return;
        }
        double dF, dL, dW;
        if (move instanceof RobotCommand.GlobalVelocity g) {
            dF = c * g.x() + sn * g.y();
            dL = -sn * g.x() + c * g.y();
            dW = g.angular();
        } else if (move instanceof RobotCommand.LocalVelocity l) {
            dF = l.forward();
            dL = l.left();
            dW = l.angular();
        } else if (move instanceof RobotCommand.WheelVelocity wv && wheels.length == 4) { // wheel surface speeds -> body speed
            double[] ws = {wv.frontRight(), wv.backRight(), wv.backLeft(), wv.frontLeft()};
            double[][] m = new double[4][3];
            for (int i = 0; i < 4; i++) {
                var dir = wheels[i].driveDirectionLocal();
                m[i] = new double[]{dir.getX(), dir.getY(), wheels[i].mountRadius};
            }
            double[] b = new org.apache.commons.math3.linear.SingularValueDecomposition(
                    org.apache.commons.math3.linear.MatrixUtils.createRealMatrix(m)).getSolver().solve(
                    new org.apache.commons.math3.linear.ArrayRealVector(ws)).toArray();
            dF = b[0];
            dL = b[1];
            dW = b[2];
        } else {
            dF = dL = dW = 0;
        }
        cmdVx = dF;
        cmdVy = dL;
        cmdW = dW;
        double k = 1.0 / specs.erVelocityTimeConstant;
        double aF = k * (dF - vF), aR = k * (-dL - vR), aW = k * (dW - w);
        aW += specs.rotationError * aF;
        // limit through the wheel coupling (speed-up / brake limits per wheel, like ER-Force's limitAcceleration)
        double[] speed = {vR, vF, w};
        double[] wheelSpeed = coupling.operate(speed);
        double[] wheelAcc = coupling.operate(new double[]{aR, aF, aW});
        for (int i = 0; i < 4; i++) {
            boolean speedingUp = Math.signum(wheelAcc[i]) == Math.signum(wheelSpeed[i]) || wheelSpeed[i] == 0;
            double lim = speedingUp ? specs.erWheelAccelSpeedup : specs.erWheelAccelBrake;
            wheelAcc[i] = Math.max(-lim, Math.min(lim, wheelAcc[i]));
        }
        double[] a = couplingPinv.operate(wheelAcc); // (right, forward, yaw)
        double m = robot.mass();
        robot.applyForce(new org.apache.commons.math3.geometry.euclidean.twod.Vector2D(
                m * (a[1] * c + a[0] * sn), m * (a[1] * sn - a[0] * c)));
        robot.applyTorque(robot.inertia() * a[2]);
        syncWheels(vF, -vR, w);
    }

    /** Wheels roll without slip in this model; keep their speed consistent with the body motion (display / telemetry). */
    private void syncWheels(double vF, double vL, double w) {
        for (OmniWheel wh : wheels) wh.setKinematic(wh.surfaceSpeedFor(vF, vL, w));
    }
}
