package ssl.sim;

import ssl.sim.core.FieldGeometry;
import ssl.sim.core.SimConfig;
import ssl.sim.core.Simulator;
import ssl.sim.net.SslNetwork;

import java.nio.file.Path;

/** Entry point. Run with --help for the options. Settings come from an optional JSON file; CLI options override it. */
public final class Main {
    private Main() {}

    /** Written to ./config.json when it does not exist (network settings only; everything else keeps its default). */
    private static final String DEFAULT_CONFIG = """
            {
              "network": {
                "visionAddress": "224.5.23.2",
                "visionPort": 10020,
                "controlPort": 10300,
                "bluePort": 10301,
                "yellowPort": 10302,
                "grSimPort": 20011,
                "syncPort": 10400,
                "networkInterface": null,
                "syncTeam": "BLUE",
                "replyToRobotControl": true
              }
            }
            """;

    public static void main(String[] args) throws Exception {
        // pass 1: config file
        SimConfig cfg = new SimConfig();
        Path defaultFile = Path.of("config.json"); // loaded automatically when no --config is given
        boolean explicit = false;
        for (int i = 0; i < args.length; i++) {
            if (args[i].equals("--config")) {
                cfg = SimConfig.load(Path.of(args[++i]));
                explicit = true;
            }
        }
        if (!explicit && !java.nio.file.Files.exists(defaultFile)) {
            try {
                java.nio.file.Files.writeString(defaultFile, DEFAULT_CONFIG);
                System.out.println("Created default " + defaultFile.toAbsolutePath());
            } catch (java.io.IOException e) {
                System.err.println("Cannot create " + defaultFile + ": " + e.getMessage());
            }
        }
        if (!explicit && java.nio.file.Files.isRegularFile(defaultFile)) {
            cfg = SimConfig.load(defaultFile);
            System.out.println("Loaded " + defaultFile.toAbsolutePath());
        }
        boolean headless = false, noNet = false, demo = false;
        String telemetryDir = null;
        String shot = null;
        int robots = -1;
        SslNetwork.Config net = cfg.network;
        for (int i = 0; i < args.length; i++) {
            switch (args[i]) {
                case "--config" -> i++;
                case "--write-default-config" -> {
                    new SimConfig().save(Path.of(args[++i]));
                    System.out.println("Default config written to " + args[i]);
                    return;
                }
                case "--division" -> cfg.division = args[++i];
                case "--headless" -> headless = true;
                case "--robots" -> robots = Integer.parseInt(args[++i]);
                case "--realism" -> {
                    cfg.realism = args[++i];
                    cfg.applyRealism();
                }
                case "--cameras" -> cfg.cameras.count = Integer.parseInt(args[++i]);
                case "--camera-height" -> cfg.cameras.height = Double.parseDouble(args[++i]);
                case "--no-net" -> noNet = true;
                case "--demo" -> demo = true;
                case "--telemetry" -> telemetryDir = args[++i];
                case "--dribble-statics" -> {
                    var rs = cfg.robot;
                    var both = ssl.sim.robot.DribblerStatics.analyse(rs, new ssl.sim.robot.Dribbler(rs).rubber.mu(),
                            new ssl.sim.robot.Chipper(rs).material().mu(), cfg.ball.muSlide, cfg.ball.mass);
                    System.out.printf("top-view coverage %.1f %%, layout problems: %s%n", rs.topViewCoverage() * 100, rs.validateLayout());
                    System.out.println("three point (roller + chipper + ground): " + both.threePoint());
                    System.out.println("two point   (roller + ground)          : " + both.twoPoint());
                    return;
                }
                case "--screenshot" -> shot = args[++i];
                case "--vision-ip" -> net.visionAddress = args[++i];
                case "--vision-port" -> net.visionPort = Integer.parseInt(args[++i]);
                case "--interface" -> net.networkInterface = args[++i];
                case "--control-port" -> net.controlPort = Integer.parseInt(args[++i]);
                case "--blue-port" -> net.bluePort = Integer.parseInt(args[++i]);
                case "--yellow-port" -> net.yellowPort = Integer.parseInt(args[++i]);
                case "--grsim-port" -> net.grSimPort = Integer.parseInt(args[++i]);
                case "--sync-port" -> net.syncPort = Integer.parseInt(args[++i]);
                default -> {
                    System.out.println("""
                            RoboDragons SSL Simulator
                              --config FILE             JSON settings (Gson); see --write-default-config
                              --write-default-config F  write a complete default config file and exit
                              --division A|B            field size (A: 12x9 m, B: 9x6 m; default B)
                              --robots N                robots per team (default: rule maximum of the division)
                              --realism NAME            ER-Force realism preset: None, Friendly, RC2021, Realistic (default Realistic)
                              --cameras N               number of cameras (default: A=2, B=1)
                              --camera-height M         camera height in metres (default 6)
                              --headless                no 3D viewer
                              --screenshot FILE         render ~90 frames, save PNG and exit
                              --no-net                  disable all UDP endpoints
                              --dribble-statics         print the static force balance of the held ball (2 and 3 point) and exit
                              --telemetry DIR           log received robot commands and robot states as CSV
                              --demo                    blue robot 0 picks up the ball with the dribbler, then kicks
                              --vision-ip/--vision-port, --interface <ip|name>
                              --control-port --blue-port --yellow-port --grsim-port --sync-port""");
                    return;
                }
            }
        }
        int perTeam = robots > 0 ? robots : cfg.robotsPerTeam > 0 ? cfg.robotsPerTeam : cfg.divisionEnum().maxRobots;
        FieldGeometry field = cfg.field();
        Simulator sim = new Simulator(field, cfg);
        sim.setupTeams(perTeam);
        if (telemetryDir != null) sim.setTelemetry(new ssl.sim.core.Telemetry(Path.of(telemetryDir)));
        SslNetwork network = null;
        if (!noNet) {
            network = new SslNetwork(sim, net);
            network.start();
        }
        if (demo) Demo.start(sim);
        sim.start();
        if (headless) {
            System.out.println("Simulator running headless (Ctrl+C to stop).");
            Thread.currentThread().join();
        } else {
            var viewer = new ssl.sim.viewer.Viewer(sim);
            if (shot != null) viewer.screenshot(shot).cameraPreset(demo ? 4 : 2);
            if (demo) viewer.selectRobot(ssl.sim.robot.Robot.Team.BLUE, 0).cameraPreset(4);
            viewer.run();
            sim.stop();
        }
        if (network != null) network.close();
    }
}
