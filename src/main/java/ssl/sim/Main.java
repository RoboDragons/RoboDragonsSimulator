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

    /** Loads {@code option FILE} (or ./defaultName, which is created from {@code defaultText} if missing) and applies it. */
    private static void loadParamFile(String[] args, String option, String defaultName, String defaultText,
                                      java.util.function.Consumer<String> apply) throws java.io.IOException {
        Path file = Path.of(defaultName);
        boolean explicit = false;
        for (int i = 0; i < args.length - 1; i++) {
            if (args[i].equals(option)) {
                file = Path.of(args[i + 1]);
                explicit = true;
            }
        }
        if (!explicit && !java.nio.file.Files.exists(file)) {
            try {
                java.nio.file.Files.writeString(file, defaultText);
                System.out.println("Created default " + file.toAbsolutePath());
            } catch (java.io.IOException e) {
                System.err.println("Cannot create " + file + ": " + e.getMessage());
            }
        }
        if (java.nio.file.Files.isRegularFile(file)) {
            apply.accept(java.nio.file.Files.readString(file, java.nio.charset.StandardCharsets.UTF_8));
            System.out.println("Loaded " + file.toAbsolutePath());
        } else if (explicit) {
            throw new java.io.FileNotFoundException(file.toString());
        }
    }

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
        // robot / ball parameters: own commented YAML files, applied over config.json (created if missing)
        final SimConfig cfg0 = cfg;
        loadParamFile(args, "--robot-specs", "robot_specs.yaml", ssl.sim.core.RobotSpecsYaml.defaultYaml(),
                text -> ssl.sim.core.RobotSpecsYaml.apply(text, cfg0.robot));
        loadParamFile(args, "--ball-params", "ball_params.yaml", ssl.sim.core.BallParamsYaml.defaultYaml(),
                text -> ssl.sim.core.BallParamsYaml.apply(text, cfg0.ball));
        loadParamFile(args, "--field-params", "field_params.yaml", ssl.sim.core.FieldParamsYaml.defaultYaml(cfg0.divisionEnum()), text -> {
            ssl.sim.core.FieldParams p = ssl.sim.core.FieldParams.defaultsFor(cfg0.divisionEnum());
            ssl.sim.core.FieldParamsYaml.apply(text, p);
            cfg0.fieldParams = p;
        });
        loadParamFile(args, "--camera-params", "camera_params.yaml", ssl.sim.core.CameraParamsYaml.defaultYaml(cfg0.field()),
                text -> ssl.sim.core.CameraParamsYaml.apply(text, cfg0.cameras));
        boolean headless = false, noNet = false, demo = false, robotView = false;
        String telemetryDir = null;
        String shot = null;
        int robots = -1;
        SslNetwork.Config net = cfg.network;
        for (int i = 0; i < args.length; i++) {
            switch (args[i]) {
                case "--config", "--robot-specs", "--ball-params", "--camera-params", "--field-params" -> i++;
                case "--write-default-field-params" -> {
                    java.nio.file.Files.writeString(Path.of(args[++i]), ssl.sim.core.FieldParamsYaml.defaultYaml(cfg.divisionEnum()));
                    System.out.println("Default field params written to " + args[i]);
                    return;
                }
                case "--write-default-camera-params" -> {
                    java.nio.file.Files.writeString(Path.of(args[++i]), ssl.sim.core.CameraParamsYaml.defaultYaml(cfg.field()));
                    System.out.println("Default camera params written to " + args[i]);
                    return;
                }
                case "--write-default-ball-params" -> {
                    java.nio.file.Files.writeString(Path.of(args[++i]), ssl.sim.core.BallParamsYaml.defaultYaml());
                    System.out.println("Default ball params written to " + args[i]);
                    return;
                }
                case "--write-default-robot-specs" -> {
                    java.nio.file.Files.writeString(Path.of(args[++i]), ssl.sim.core.RobotSpecsYaml.defaultYaml());
                    System.out.println("Default robot specs written to " + args[i]);
                    return;
                }
                case "--write-default-config" -> {
                    new SimConfig().save(Path.of(args[++i]));
                    System.out.println("Default config written to " + args[i]);
                    return;
                }
                case "--division" -> {
                    cfg.division = args[++i];
                    cfg.fieldParams = null; // defaults of that division
                    cfg.cameras.positions = null; // explicit camera positions belong to one field size: use the automatic layout
                }
                case "--headless" -> headless = true;
                case "--robots" -> robots = Integer.parseInt(args[++i]);
                case "--realism" -> {
                    cfg.realism = args[++i];
                    cfg.applyRealism();
                }
                case "--cameras" -> {
                    cfg.cameras.count = Integer.parseInt(args[++i]);
                    cfg.cameras.positions = null; // automatic layout with this many cameras
                }
                case "--camera-height" -> {
                    cfg.cameras.height = Double.parseDouble(args[++i]);
                    cfg.cameras.positions = null; // automatic layout at this height
                }
                case "--no-net" -> noNet = true;
                case "--demo" -> demo = true;
                case "--robot-view" -> robotView = true;
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
                              --robot-specs FILE        robot parameters + ball contact materials (YAML, default ./robot_specs.yaml, created if missing)
                              --ball-params FILE        ball parameters (YAML, default ./ball_params.yaml, created if missing)
                              --field-params FILE       field dimensions, margins, goal side walls (YAML, default ./field_params.yaml, created if missing)
                              --write-default-field-params F  write the field parameter file of the selected division and exit
                              --camera-params FILE      camera parameters and positions (YAML, default ./camera_params.yaml, created if missing)
                              --write-default-camera-params F  write the default camera parameter file and exit
                              (--division selects the division defaults instead of field_params.yaml; --division, --cameras and --camera-height select the automatic camera layout instead of camera_params.yaml positions)
                              --write-default-ball-params F  write the default ball parameter file and exit
                              --write-default-robot-specs F  write the default robot parameter file and exit
                              --division A|B            field size (A: 12x9 m, B: 9x6 m; default A)
                              --robots N                robots per team (default: rule maximum of the division)
                              --realism NAME            ER-Force realism preset: None, Friendly, RC2021, Realistic (default Realistic)
                              --cameras N               number of cameras (default: A=2, B=1)
                              --camera-height M         camera height in metres (default 6)
                              --headless                no 3D viewer
                              --screenshot FILE         render ~90 frames, save PNG and exit
                              --no-net                  disable all UDP endpoints
                              --dribble-statics         print the static force balance of the held ball (2 and 3 point) and exit
                              --telemetry DIR           log received robot commands and robot states as CSV
                              --robot-view                close-up of blue robot 0 (with --screenshot, e.g. for an icon)
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
            if (shot != null) viewer.screenshot(shot).cameraPreset(robotView ? 5 : demo ? 4 : 2);
            if (robotView) viewer.selectRobot(ssl.sim.robot.Robot.Team.BLUE, 0).cameraPreset(5);
            if (demo) viewer.selectRobot(ssl.sim.robot.Robot.Team.BLUE, 0).cameraPreset(4);
            viewer.run();
            sim.stop();
        }
        if (network != null) network.close();
    }
}
