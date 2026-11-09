package ssl.sim.net;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import robo_dragons_cafe.network.proto.ssl.grsim.GrsimCommand;
import robo_dragons_cafe.network.proto.ssl.grsim.GrsimPacket;
import robo_dragons_cafe.network.proto.ssl.simulation.SslGcCommon;
import robo_dragons_cafe.network.proto.ssl.simulation.SslSimulationConfig;
import robo_dragons_cafe.network.proto.ssl.simulation.SslSimulationControl;
import robo_dragons_cafe.network.proto.ssl.simulation.SslSimulationError;
import robo_dragons_cafe.network.proto.ssl.simulation.SslSimulationRobotControl;
import robo_dragons_cafe.network.proto.ssl.simulation.SslSimulationRobotFeedback;
import robo_dragons_cafe.network.proto.ssl.simulation.SslVisionGeometry;
import ssl.sim.core.FieldGeometry;
import ssl.sim.core.Simulator;
import ssl.sim.core.World;
import ssl.sim.robot.Robot;
import ssl.sim.robot.RobotCommand;
import ssl.sim.robot.RobotSpecs;

import java.util.ArrayList;
import java.util.List;
import java.util.function.IntConsumer;

/** Applies the messages of the SSL simulation protocol (and legacy grSim packets) to the simulator. */
public final class SimCommands {
    private final Simulator sim;
    /** Called when the config requests another vision port. */
    public IntConsumer visionPortChanged = p -> {};
    /** Probability that one robot command is dropped. */
    public double commandLoss = 0.0;
    private final java.util.Random random = new java.util.Random(1);

    public SimCommands(Simulator sim) { this.sim = sim; }

    private static Robot.Team team(SslGcCommon.Team t) {
        return t == SslGcCommon.Team.YELLOW ? Robot.Team.YELLOW : t == SslGcCommon.Team.BLUE ? Robot.Team.BLUE : null;
    }

    private static SslSimulationError.SimulatorError error(String code, String msg) {
        return SslSimulationError.SimulatorError.newBuilder().setCode(code).setMessage(msg).build();
    }

    // ------------------------------------------------------------ simulator control / config

    public SslSimulationControl.SimulatorResponse handle(SslSimulationControl.SimulatorCommand cmd) {
        List<SslSimulationError.SimulatorError> errors = new ArrayList<>();
        World w = sim.world();
        w.lock().lock();
        try {
            if (cmd.hasConfig()) applyConfig(cmd.getConfig(), errors);
            if (cmd.hasControl()) applyControl(cmd.getControl(), errors);
        } finally {
            w.lock().unlock();
        }
        return SslSimulationControl.SimulatorResponse.newBuilder().addAllErrors(errors).build();
    }

    private void applyControl(SslSimulationControl.SimulatorControl c, List<SslSimulationError.SimulatorError> errors) {
        World w = sim.world();
        if (c.hasSimulationSpeed()) sim.setSpeed(c.getSimulationSpeed());
        if (c.hasTeleportBall()) {
            SslSimulationControl.TeleportBall tb = c.getTeleportBall();
            if (tb.getByForce()) errors.add(error("UNSUPPORTED", "TeleportBall.by_force is not supported; ball was not moved"));
            else if (tb.hasX() || tb.hasY() || tb.hasZ() || tb.hasVx() || tb.hasVy() || tb.hasVz()) {
                var b = w.ball();
                Vector3D p = new Vector3D(tb.hasX() ? tb.getX() : b.x(), tb.hasY() ? tb.getY() : b.y(),
                        tb.hasZ() ? tb.getZ() : b.z());
                Vector3D v = new Vector3D(tb.hasVx() ? tb.getVx() : 0, tb.hasVy() ? tb.getVy() : 0, tb.hasVz() ? tb.getVz() : 0);
                if (tb.getTeleportSafely()) clearRobotsAround(p.getX(), p.getY());
                b.teleport(p, v, tb.getRoll());
            }
        }
        for (SslSimulationControl.TeleportRobot tr : c.getTeleportRobotList()) {
            Robot.Team t = team(tr.getId().getTeam());
            if (t == null) {
                errors.add(error("INVALID_TEAM", "TeleportRobot without team"));
                continue;
            }
            int id = tr.getId().getId();
            if (tr.getByForce()) {
                errors.add(error("UNSUPPORTED", "TeleportRobot.by_force is not supported"));
                continue;
            }
            Robot r = w.findRobot(t, id);
            if (tr.hasPresent() && !tr.getPresent()) {
                w.removeRobot(t, id);
                continue;
            }
            if (r == null) {
                if (!tr.hasPresent() || !tr.getPresent()) {
                    errors.add(error("UNKNOWN_ROBOT", "Robot " + t + " " + id + " is not present"));
                    continue;
                }
                r = w.addRobot(t, id, sim.specsFor(t, id), 0, 0, 0);
            }
            double x = tr.hasX() ? tr.getX() : r.x(), y = tr.hasY() ? tr.getY() : r.y();
            double o = tr.hasOrientation() ? tr.getOrientation() : r.heading();
            r.setPose(x, y, o);
            r.setVelocity(tr.getVX(), tr.getVY(), tr.getVAngular());
        }
    }

    private void clearRobotsAround(double x, double y) {
        for (Robot r : sim.world().robots()) {
            double dx = r.x() - x, dy = r.y() - y, d = Math.hypot(dx, dy);
            double need = r.specs().radius + 0.1;
            if (d < need) {
                double ux = d < 1e-6 ? 1 : dx / d, uy = d < 1e-6 ? 0 : dy / d;
                r.setPose(x + ux * need, y + uy * need, r.heading());
                r.setVelocity(0, 0, 0);
            }
        }
    }

    private void applyConfig(SslSimulationConfig.SimulatorConfig cfg, List<SslSimulationError.SimulatorError> errors) {
        if (cfg.hasGeometry() && cfg.getGeometry().hasField()) {
            SslVisionGeometry.SSL_GeometryFieldSize f = cfg.getGeometry().getField();
            FieldGeometry cur = sim.world().field();
            FieldGeometry nf = new FieldGeometry(cur.division, f.getFieldLength() / 1000.0, f.getFieldWidth() / 1000.0,
                    f.getGoalWidth() / 1000.0,
                    f.hasPenaltyAreaDepth() ? f.getPenaltyAreaDepth() / 1000.0 : cur.penaltyDepth,
                    f.hasPenaltyAreaWidth() ? f.getPenaltyAreaWidth() / 1000.0 : cur.penaltyWidth);
            sim.setField(nf); // the world lock is reentrant
        }
        for (SslSimulationConfig.RobotSpecs rs : cfg.getRobotSpecsList()) {
            Robot.Team t = team(rs.getId().getTeam());
            if (t == null) {
                errors.add(error("INVALID_TEAM", "RobotSpecs without team"));
                continue;
            }
            int id = rs.getId().getId();
            RobotSpecs s = sim.specsFor(t, id);
            if (rs.hasRadius()) s.radius = rs.getRadius();
            if (rs.hasHeight()) s.height = rs.getHeight();
            if (rs.hasMass()) s.mass = rs.getMass();
            s.inertia = 0;
            if (rs.hasMaxLinearKickSpeed()) s.maxLinearKickSpeed = rs.getMaxLinearKickSpeed();
            if (rs.hasMaxChipKickSpeed()) s.maxChipKickSpeed = rs.getMaxChipKickSpeed();
            if (rs.hasCenterToDribbler()) s.centerToDribbler = rs.getCenterToDribbler();
            if (s.centerToDribbler >= s.radius) {
                errors.add(error("INVALID_SPECS", "center_to_dribbler must be smaller than radius"));
                continue;
            }
            if (rs.hasLimits()) {
                var l = rs.getLimits();
                if (l.hasAccSpeedupAbsoluteMax()) s.accSpeedupAbsoluteMax = l.getAccSpeedupAbsoluteMax();
                if (l.hasAccSpeedupAngularMax()) s.accSpeedupAngularMax = l.getAccSpeedupAngularMax();
                if (l.hasAccBrakeAbsoluteMax()) s.accBrakeAbsoluteMax = l.getAccBrakeAbsoluteMax();
                if (l.hasAccBrakeAngularMax()) s.accBrakeAngularMax = l.getAccBrakeAngularMax();
                if (l.hasVelAbsoluteMax()) s.velAbsoluteMax = l.getVelAbsoluteMax();
                if (l.hasVelAngularMax()) s.velAngularMax = l.getVelAngularMax();
            }
            if (rs.hasWheelAngles()) { // protocol angles are clockwise, internal ones counter-clockwise
                var a = rs.getWheelAngles();
                s.wheelAngles = new double[]{-a.getFrontRight(), -a.getBackRight(), -a.getBackLeft(), -a.getFrontLeft()};
            }
            sim.setSpecs(t, id, s);
            Robot old = sim.world().findRobot(t, id);
            if (old != null) {
                Robot n = sim.world().addRobot(t, id, s, old.x(), old.y(), old.heading());
                n.setVelocity(old.velocity().getX(), old.velocity().getY(), old.angularVelocity());
            }
        }
        if (cfg.hasVisionPort()) visionPortChanged.accept(cfg.getVisionPort());
    }

    // ------------------------------------------------------------ robot control

    public SslSimulationRobotFeedback.RobotControlResponse handle(Robot.Team team, SslSimulationRobotControl.RobotControl rc) {
        SslSimulationRobotFeedback.RobotControlResponse.Builder resp = SslSimulationRobotFeedback.RobotControlResponse.newBuilder();
        World w = sim.world();
        w.lock().lock();
        try {
            for (SslSimulationRobotControl.RobotCommand c : rc.getRobotCommandsList()) {
                if (commandLoss > 0 && random.nextDouble() < commandLoss) continue;
                Robot r = w.findRobot(team, c.getId());
                if (r == null) {
                    resp.addErrors(error("UNKNOWN_ROBOT", "Robot " + team + " " + c.getId() + " is not on the field"));
                    continue;
                }
                RobotCommand.Move move = null;
                if (c.hasMoveCommand()) {
                    var m = c.getMoveCommand();
                    switch (m.getCommandCase()) {
                        case WHEEL_VELOCITY -> {
                            var v = m.getWheelVelocity();
                            move = new RobotCommand.WheelVelocity(v.getFrontRight(), v.getBackRight(), v.getBackLeft(), v.getFrontLeft());
                        }
                        case LOCAL_VELOCITY -> {
                            var v = m.getLocalVelocity();
                            move = new RobotCommand.LocalVelocity(v.getForward(), v.getLeft(), v.getAngular());
                        }
                        case GLOBAL_VELOCITY -> {
                            var v = m.getGlobalVelocity();
                            move = new RobotCommand.GlobalVelocity(v.getX(), v.getY(), v.getAngular());
                        }
                        default -> { }
                    }
                }
                RobotCommand rcmd = new RobotCommand(move, c.getKickSpeed(), c.getKickAngle(), c.getDribblerSpeed());
                r.applyCommand(rcmd);
                if (sim.telemetry() != null) sim.telemetry().command(w.time(), team, c.getId(), rcmd);
                resp.addFeedback(SslSimulationRobotFeedback.RobotFeedback.newBuilder().setId(c.getId())
                        .setDribblerBallContact(r.dribbler().hasBallContact()));
            }
        } finally {
            w.lock().unlock();
        }
        return resp.build();
    }

    // ------------------------------------------------------------ legacy grSim

    public void handle(GrsimPacket.Packet p) {
        World w = sim.world();
        w.lock().lock();
        try {
            if (p.hasCommands()) {
                var cmds = p.getCommands();
                Robot.Team team = cmds.getIsteamyellow() ? Robot.Team.YELLOW : Robot.Team.BLUE;
                for (GrsimCommand.Command c : cmds.getRobotCommandsList()) {
                    Robot r = w.findRobot(team, c.getId());
                    if (r == null) continue;
                    RobotCommand.Move move;
                    if (c.getWheelsspeed()) { // grSim wheel order (CCW from front-left): 1=FL 2=BL 3=BR 4=FR, rad/s
                        double rad = r.specs().wheelRadius;
                        move = new RobotCommand.WheelVelocity(c.getWheel4() * rad, c.getWheel3() * rad,
                                c.getWheel2() * rad, c.getWheel1() * rad);
                    } else {
                        move = new RobotCommand.LocalVelocity(c.getVeltangent(), c.getVelnormal(), c.getVelangular());
                    }
                    double kx = c.getKickspeedx(), kz = c.getKickspeedz();
                    double speed = Math.hypot(kx, kz);
                    double angle = kz > 0 ? Math.toDegrees(Math.atan2(kz, kx)) : 0;
                    r.applyCommand(new RobotCommand(move, speed, angle, c.getSpinner() ? 9000 : 0));
                }
            }
            if (p.hasReplacement()) {
                var rep = p.getReplacement();
                if (rep.hasBall()) {
                    var b = rep.getBall();
                    w.ball().teleport(new Vector3D(b.getX(), b.getY(), w.ball().radius()), new Vector3D(b.getVx(), b.getVy(), 0), true);
                }
                for (var rr : rep.getRobotsList()) {
                    Robot.Team t = rr.getYellowteam() ? Robot.Team.YELLOW : Robot.Team.BLUE;
                    if (rr.hasTurnon() && !rr.getTurnon()) {
                        w.removeRobot(t, rr.getId());
                        continue;
                    }
                    Robot r = w.findRobot(t, rr.getId());
                    if (r == null) r = w.addRobot(t, rr.getId(), sim.specsFor(t, rr.getId()), 0, 0, 0);
                    r.setPose(rr.getX(), rr.getY(), rr.getDir());
                    r.setVelocity(0, 0, 0);
                }
            }
        } finally {
            w.lock().unlock();
        }
    }
}
