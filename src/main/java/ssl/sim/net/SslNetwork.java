package ssl.sim.net;

import com.google.protobuf.InvalidProtocolBufferException;
import robo_dragons_cafe.network.proto.ssl.grsim.GrsimPacket;
import robo_dragons_cafe.network.proto.ssl.simulation.SslSimulationControl;
import robo_dragons_cafe.network.proto.ssl.simulation.SslSimulationRobotFeedback;
import robo_dragons_cafe.network.proto.ssl.simulation.SslSimulationRobotControl;
import robo_dragons_cafe.network.proto.ssl.simulation.SslSimulationSynchronous;
import ssl.sim.core.Simulator;
import ssl.sim.robot.Robot;
import ssl.sim.vision.Camera;

import java.io.IOException;
import java.net.*;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * All UDP endpoints of the simulator (ports follow the SSL conventions / ssl-simulation-protocol):
 * <ul>
 *   <li>vision: multicast 224.5.23.2:10020 ({@code SSL_WrapperPacket}: detection per camera + geometry at 1 Hz)</li>
 *   <li>simulator control 10300 ({@code SimulatorCommand} -> {@code SimulatorResponse})</li>
 *   <li>blue robot control 10301, yellow robot control 10302 ({@code RobotControl} -> {@code RobotControlResponse})</li>
 *   <li>legacy grSim packets 20011 ({@code grSim_Packet})</li>
 *   <li>synchronous mode 10400 ({@code SimulationSyncRequest} -> {@code SimulationSyncResponse})</li>
 * </ul>
 */
public class SslNetwork implements AutoCloseable {
    public static class Config {
        public String visionAddress = "224.5.23.2";
        public int visionPort = 10020;
        public int controlPort = 10300, bluePort = 10301, yellowPort = 10302, grSimPort = 20011, syncPort = 10400;
        public String networkInterface = null; // IP address or interface name used for multicast
        public Robot.Team syncTeam = Robot.Team.BLUE;
        /** Answer robot control packets with a RobotControlResponse (feedback), as the protocol specifies. */
        public boolean replyToRobotControl = true;
        /** Probability that a single robot command is lost (ER-Force: robot_command_loss). */
        public double robotCommandLoss = 0.0;
        /** Delay between receiving a robot control packet and applying it (ER-Force: command_delay). */
        public double commandDelaySeconds = 0.0;
    }

    private final Simulator sim;
    private final Config cfg;
    private final SimCommands commands;
    private final java.util.concurrent.ScheduledExecutorService delayed = java.util.concurrent.Executors.newSingleThreadScheduledExecutor(r -> {
        Thread t = new Thread(r, "command-delay");
        t.setDaemon(true);
        return t;
    });
    private final List<Thread> threads = new ArrayList<>();
    private final List<DatagramSocket> sockets = new ArrayList<>();
    private final AtomicBoolean running = new AtomicBoolean(true);
    private MulticastSocket sender;
    private volatile int visionPort;
    private volatile long lastGeometry;

    public SslNetwork(Simulator sim, Config cfg) {
        this.sim = sim;
        this.cfg = cfg;
        this.visionPort = cfg.visionPort;
        this.commands = new SimCommands(sim);
        this.commands.commandLoss = cfg.robotCommandLoss;
        commands.visionPortChanged = p -> visionPort = p;
    }

    public void start() {
        try {
            sender = new MulticastSocket();
            sender.setTimeToLive(1);
            NetworkInterface ni = null;
            if (cfg.networkInterface != null) {
                ni = NetworkInterface.getByName(cfg.networkInterface);
                if (ni == null) ni = NetworkInterface.getByInetAddress(InetAddress.getByName(cfg.networkInterface));
            } else {
                ni = localInterface();
            }
            if (ni != null) {
                sender.setNetworkInterface(ni);
                System.out.println("[net] multicast via interface " + ni.getDisplayName() + " " + ni.inetAddresses().toList());
            }
        } catch (IOException e) {
            System.err.println("[net] cannot create multicast sender: " + e);
        }
        sim.addFrameListener(this::publishVision);

        listen(cfg.controlPort, "sim-control", (data, from, sock) -> {
            var cmd = SslSimulationControl.SimulatorCommand.parseFrom(data);
            reply(sock, from, commands.handle(cmd).toByteArray());
        });
        listen(cfg.bluePort, "blue-control", (data, from, sock) -> robotControl(Robot.Team.BLUE, data, from, sock));
        listen(cfg.yellowPort, "yellow-control", (data, from, sock) -> robotControl(Robot.Team.YELLOW, data, from, sock));
        listen(cfg.grSimPort, "grsim", (data, from, sock) -> commands.handle(GrsimPacket.Packet.parseFrom(data)));
        listen(cfg.syncPort, "sync", this::sync);
    }

    /**
     * The local (LAN) interface used for multicast when none is configured: up, multicast capable, not loopback and not
     * a virtual adapter (Hyper-V / WSL / VMware / VirtualBox / Docker), with an IPv4 address.
     */
    static NetworkInterface localInterface() {
        try {
            for (NetworkInterface ni : java.util.Collections.list(NetworkInterface.getNetworkInterfaces())) {
                String n = (ni.getDisplayName() + " " + ni.getName()).toLowerCase();
                if (!ni.isUp() || ni.isLoopback() || ni.isVirtual() || !ni.supportsMulticast()) continue;
                if (n.contains("vethernet") || n.contains("hyper-v") || n.contains("vmware") || n.contains("virtualbox")
                        || n.contains("docker") || n.contains("wsl") || n.contains("bluetooth")) continue;
                if (ni.inetAddresses().anyMatch(a -> a instanceof java.net.Inet4Address && a.isSiteLocalAddress())) return ni;
            }
        } catch (SocketException ignored) {
        }
        return null;
    }

    private void robotControl(Robot.Team team, byte[] data, SocketAddress from, DatagramSocket sock) throws IOException {
        var rc = SslSimulationRobotControl.RobotControl.parseFrom(data);
        if (cfg.commandDelaySeconds > 0) {
            delayed.schedule(() -> commands.handle(team, rc), (long) (cfg.commandDelaySeconds * 1e6), java.util.concurrent.TimeUnit.MICROSECONDS);
            if (cfg.replyToRobotControl) reply(sock, from, SslSimulationRobotFeedback.RobotControlResponse.getDefaultInstance().toByteArray());
            return;
        }
        var response = commands.handle(team, rc);
        if (cfg.replyToRobotControl) reply(sock, from, response.toByteArray());
    }

    private void sync(byte[] data, SocketAddress from, DatagramSocket sock) throws IOException {
        var req = SslSimulationSynchronous.SimulationSyncRequest.parseFrom(data);
        SslSimulationSynchronous.SimulationSyncResponse.Builder resp = SslSimulationSynchronous.SimulationSyncResponse.newBuilder();
        if (req.hasSimulatorCommand()) commands.handle(req.getSimulatorCommand());
        if (req.hasRobotControl()) resp.setRobotControlResponse(commands.handle(cfg.syncTeam, req.getRobotControl()));
        double step = req.hasSimStep() ? req.getSimStep() : 1.0 / 60;
        for (Camera.DetectionFrame f : sim.advance(step)) resp.addDetection(ProtoConvert.simDetectionFrame(f));
        reply(sock, from, resp.build().toByteArray());
    }

    private interface Handler {
        void handle(byte[] data, SocketAddress from, DatagramSocket sock) throws IOException;
    }

    private void listen(int port, String name, Handler h) {
        DatagramSocket sock;
        try {
            sock = new DatagramSocket(null);
            sock.setReuseAddress(true);
            sock.bind(new InetSocketAddress(port));
        } catch (IOException e) {
            System.err.println("[net] cannot bind UDP port " + port + " (" + name + "): " + e.getMessage());
            return;
        }
        sockets.add(sock);
        Thread t = new Thread(() -> {
            byte[] buf = new byte[65536];
            while (running.get()) {
                try {
                    DatagramPacket p = new DatagramPacket(buf, buf.length);
                    sock.receive(p);
                    byte[] data = java.util.Arrays.copyOf(p.getData(), p.getLength());
                    h.handle(data, p.getSocketAddress(), sock);
                } catch (InvalidProtocolBufferException e) {
                    System.err.println("[net] " + name + ": invalid message: " + e.getMessage());
                } catch (IOException e) {
                    if (running.get() && !sock.isClosed()) System.err.println("[net] " + name + ": " + e);
                }
            }
        }, "udp-" + name);
        t.setDaemon(true);
        t.start();
        threads.add(t);
        System.out.println("[net] listening on UDP " + port + " (" + name + ")");
    }

    private static void reply(DatagramSocket sock, SocketAddress to, byte[] data) throws IOException {
        sock.send(new DatagramPacket(data, data.length, to));
    }

    private void multicast(String addr, int port, byte[] data) {
        if (sender == null) return;
        try {
            sender.send(new DatagramPacket(data, data.length, InetAddress.getByName(addr), port));
        } catch (IOException e) {
            // network may be down; ignore silently to keep the simulation going
        }
    }

    private void publishVision(Camera.DetectionFrame f) {
        multicast(cfg.visionAddress, visionPort, ProtoConvert.detectionPacket(f).toByteArray());
        long now = System.nanoTime();
        if (now - lastGeometry > 1_000_000_000L) {
            lastGeometry = now;
            var w = sim.world();
            multicast(cfg.visionAddress, visionPort,
                    ProtoConvert.geometryPacket(w.field(), sim.vision(), w.ball().params).toByteArray());
        }
    }

    @Override
    public void close() {
        running.set(false);
        for (DatagramSocket s : sockets) s.close();
        if (sender != null) sender.close();
        threads.forEach(Thread::interrupt);
    }
}
