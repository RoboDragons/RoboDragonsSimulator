package ssl.sim;

import org.junit.jupiter.api.Test;
import robo_dragons_cafe.network.proto.ssl.simulation.*;
import robo_dragons_cafe.network.proto.ssl.vision.VisionWrapper;
import ssl.sim.core.FieldGeometry;
import ssl.sim.core.Simulator;
import ssl.sim.net.SslNetwork;
import ssl.sim.robot.Robot;

import java.net.*;

import static org.junit.jupiter.api.Assertions.*;

class NetworkTest {
    @Test
    void robotControlOverUdpAndSyncMode() throws Exception {
        Simulator sim = new Simulator(new FieldGeometry(FieldGeometry.Division.A));
        sim.setupTeams(3);
        SslNetwork.Config cfg = new SslNetwork.Config();
        cfg.controlPort = 21300; cfg.bluePort = 21301; cfg.yellowPort = 21302; cfg.grSimPort = 21011; cfg.syncPort = 21400;
        cfg.visionPort = 21020;
        try (SslNetwork net = new SslNetwork(sim, cfg)) {
            net.start();
            try (DatagramSocket s = new DatagramSocket()) {
                s.setSoTimeout(2000);
                var rc = SslSimulationRobotControl.RobotControl.newBuilder().addRobotCommands(
                        SslSimulationRobotControl.RobotCommand.newBuilder().setId(0).setMoveCommand(
                                SslSimulationRobotControl.RobotMoveCommand.newBuilder().setLocalVelocity(
                                        SslSimulationRobotControl.MoveLocalVelocity.newBuilder().setForward(1.0f).setLeft(0).setAngular(0)))
                                .setDribblerSpeed(5000)).build();
                byte[] d = rc.toByteArray();
                s.send(new DatagramPacket(d, d.length, InetAddress.getLoopbackAddress(), cfg.bluePort));
                byte[] buf = new byte[4096];
                DatagramPacket p = new DatagramPacket(buf, buf.length);
                s.receive(p);
                var resp = SslSimulationRobotFeedback.RobotControlResponse.parseFrom(java.util.Arrays.copyOf(buf, p.getLength()));
                assertEquals(1, resp.getFeedbackCount());

                // synchronous mode: advance 1 s in 60 steps
                SslSimulationSynchronous.SimulationSyncResponse last = null;
                for (int i = 0; i < 60; i++) {
                    var req = SslSimulationSynchronous.SimulationSyncRequest.newBuilder().setSimStep(1f / 60).setRobotControl(rc).build();
                    byte[] rb = req.toByteArray();
                    s.send(new DatagramPacket(rb, rb.length, InetAddress.getLoopbackAddress(), cfg.syncPort));
                    s.receive(p);
                    last = SslSimulationSynchronous.SimulationSyncResponse.parseFrom(java.util.Arrays.copyOf(buf, p.getLength()));
                }
                assertTrue(last.getDetectionCount() > 0, "cameras should deliver detections");
                Robot r = sim.world().findRobot(Robot.Team.BLUE, 0);
                assertTrue(r.velocity().getX() > 0.5, "robot should be driving: " + r.velocity());
                // each of the 4 cameras produced a frame for the last step; a blue robot must be seen somewhere
                int seen = 0;
                for (var fr : last.getDetectionList()) seen += fr.getRobotsBlueCount();
                assertTrue(seen >= 3);

                // simulator control: teleport ball
                var cmd = SslSimulationControl.SimulatorCommand.newBuilder().setControl(
                        SslSimulationControl.SimulatorControl.newBuilder().setTeleportBall(
                                SslSimulationControl.TeleportBall.newBuilder().setX(1f).setY(2f).setVx(0).setVy(0))).build();
                byte[] cb = cmd.toByteArray();
                s.send(new DatagramPacket(cb, cb.length, InetAddress.getLoopbackAddress(), cfg.controlPort));
                s.receive(p);
                assertEquals(0, SslSimulationControl.SimulatorResponse.parseFrom(java.util.Arrays.copyOf(buf, p.getLength())).getErrorsCount());
                assertEquals(1.0, sim.world().ball().x(), 1e-6);
            }
        }
    }

    @Test
    void visionPacketsAreWellFormed() {
        Simulator sim = new Simulator(new FieldGeometry(FieldGeometry.Division.B));
        sim.setupTeams(2);
        var frames = sim.advance(0.1);
        assertFalse(frames.isEmpty());
        VisionWrapper.Packet pkt = ssl.sim.net.ProtoConvert.detectionPacket(frames.get(0));
        assertTrue(pkt.hasDetection());
        var g = ssl.sim.net.ProtoConvert.geometryPacket(sim.world().field(), sim.vision(), sim.world().ball().params);
        assertEquals(9000, g.getGeometry().getField().getFieldLength());
        assertEquals(1, g.getGeometry().getCalibCount());
        // the ball lies at the centre of the field, so some camera must project it near its principal point side
        boolean ball = false;
        for (var f : frames) ball |= !f.balls().isEmpty();
        assertTrue(ball);
    }
}
