package ssl.sim.core;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import ssl.sim.ball.Ball;
import ssl.sim.net.SslNetwork;
import ssl.sim.robot.RobotSpecs;

import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Simulator configuration, read from / written to JSON with Gson. Every field has a default, so a config
 * file only needs the entries that differ. Generate a complete file with {@code --write-default-config}.
 */
public class SimConfig {
    /** "A" or "B" (RoboCup SSL division, selects the field size). */
    public String division = "B";
    /** Robots per team at start-up; 0 = maximum for the division. */
    public int robotsPerTeam = 0;
    /**
     * Realism preset of the ER-Force simulator-cli ("None", "Friendly", "Realistic"): vision delay, noise, detection
     * dropouts, vision rate, command loss/delay and the robot rotation error. Fields given explicitly in the JSON
     * override the preset; null = no preset (the defaults of the individual fields are used).
     */
    public String realism = "Realistic";
    public CameraConfig cameras = new CameraConfig();
    /** Parameters applied to every robot (can be changed per robot at run time via the simulation protocol). */
    public RobotSpecs robot = new RobotSpecs();
    public Ball.Params ball = new Ball.Params();
    public SslNetwork.Config network = new SslNetwork.Config();

    public static class CameraConfig {
        /** Number of cameras tiled along the field length; null = 2 for division A, 1 for division B. */
        public Integer count = null;
        /** Mounting height above the field [m]. */
        public double height = 6.0;
        /** Focal length [px]; 0 = chosen automatically so that one camera covers its share of the field. */
        public double focalLengthPx = 0;
        public int imageWidth = 1280;
        public int imageHeight = 1024;
        public double frequencyHz = 60;
        public double positionNoiseMm = 1.0;
        /** Ball position noise [mm]; negative = same as positionNoiseMm. */
        public double ballNoiseMm = -1;
        public double orientationNoiseMrad = 3.0;
        public double ballDropProbability = 0.0;
        public double robotDropProbability = 0.0;
        public double latencySeconds = 0.0; // real delay before a frame is published (0 = none)
        /** Extra field area [m] around each camera's share that it still reports (overlap). */
        public double overlapMeters = 0.25;

        public int effectiveCount(FieldGeometry.Division d) {
            return Math.max(1, count != null ? count : d == FieldGeometry.Division.A ? 2 : 1);
        }
    }

    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().serializeNulls().create();

    public SimConfig() { this("Realistic"); }

    private SimConfig(String realism) {
        this.realism = realism;
        applyRealism();
    }

    /** Writes the values of the selected realism preset into the camera / network / robot settings. */
    public SimConfig applyRealism() {
        if (realism == null) return this;
        double visionDelay = 0.035, robotNoise, ballNoise, phiNoise, ballDrop, robotDrop, cmdLoss, cmdDelay, rotErr;
        switch (realism.trim().toLowerCase()) {
            case "none" -> { robotNoise = 0; ballNoise = 0; phiNoise = 0; ballDrop = 0; robotDrop = 0; cmdLoss = 0; cmdDelay = 0; rotErr = 0; }
            case "friendly" -> { robotNoise = 0.3; ballNoise = 0.4; phiNoise = 3; ballDrop = 0.05; robotDrop = 0.02; cmdLoss = 0.01; cmdDelay = 0; rotErr = 0; }
            case "rc2021" -> { robotNoise = 1.3; ballNoise = 1.0; phiNoise = 10; ballDrop = 0.05; robotDrop = 0; cmdLoss = 0.03; cmdDelay = 0; rotErr = 0; }
            case "realistic" -> { robotNoise = 1.3; ballNoise = 1.4; phiNoise = 10; ballDrop = 0.05; robotDrop = 0.02; cmdLoss = 0.03; cmdDelay = 0.003; rotErr = 0.5; }
            default -> throw new IllegalArgumentException("unknown realism preset: " + realism + " (None, Friendly, RC2021, Realistic)");
        }
        cameras.frequencyHz = 1.0 / 0.015; // ER-Force sends a vision frame every 15 ms
        cameras.latencySeconds = visionDelay;
        cameras.positionNoiseMm = robotNoise;
        cameras.ballNoiseMm = ballNoise;
        cameras.orientationNoiseMrad = phiNoise;
        cameras.ballDropProbability = ballDrop;
        cameras.robotDropProbability = robotDrop;
        network.robotCommandLoss = cmdLoss;
        network.commandDelaySeconds = cmdDelay;
        robot.rotationError = rotErr;
        return this;
    }

    public FieldGeometry.Division divisionEnum() {
        return FieldGeometry.Division.valueOf(division.trim().toUpperCase());
    }

    public FieldGeometry field() { return new FieldGeometry(divisionEnum()); }

    public static SimConfig load(Path file) throws IOException {
        return fromJson(Files.readString(file, StandardCharsets.UTF_8));
    }

    public void save(Path file) throws IOException {
        try (Writer w = Files.newBufferedWriter(file, StandardCharsets.UTF_8)) {
            GSON.toJson(this, w);
        }
    }

    public String toJson() { return GSON.toJson(this); }

    /**
     * Parses JSON on top of the realism preset it names (default "Realistic"): fields present in the JSON override the
     * preset values, all others keep them.
     */
    public static SimConfig fromJson(String json) {
        com.google.gson.JsonElement root = com.google.gson.JsonParser.parseString(json);
        String name = "Realistic";
        if (root.isJsonObject() && root.getAsJsonObject().has("realism")) {
            var e = root.getAsJsonObject().get("realism");
            name = e.isJsonNull() ? null : e.getAsString();
        }
        SimConfig base = new SimConfig(name);
        Gson g = new GsonBuilder().setPrettyPrinting().serializeNulls()
                .registerTypeAdapter(SimConfig.class, (com.google.gson.InstanceCreator<SimConfig>) t -> base)
                .registerTypeAdapter(CameraConfig.class, (com.google.gson.InstanceCreator<CameraConfig>) t -> base.cameras)
                .registerTypeAdapter(RobotSpecs.class, (com.google.gson.InstanceCreator<RobotSpecs>) t -> base.robot)
                .registerTypeAdapter(SslNetwork.Config.class, (com.google.gson.InstanceCreator<SslNetwork.Config>) t -> base.network)
                .registerTypeAdapter(Ball.Params.class, (com.google.gson.InstanceCreator<Ball.Params>) t -> base.ball)
                .create();
        SimConfig c = g.fromJson(json, SimConfig.class);
        return c != null ? c : base;
    }
}
