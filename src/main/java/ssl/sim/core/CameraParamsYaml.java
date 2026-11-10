package ssl.sim.core;

import ssl.sim.vision.VisionSystem;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

/**
 * YAML representation of {@link SimConfig.CameraConfig}: commented default file (with the camera positions of the
 * automatic layout written out) and loader (SnakeYAML).
 */
public final class CameraParamsYaml {
    private CameraParamsYaml() {}

    /** Not written: these come from the realism preset ({@code count} is given by the number of positions). */
    private static final Set<String> EXCLUDED = Set.of("count", "frequencyHz", "latencySeconds", "positionNoiseMm",
            "ballNoiseMm", "orientationNoiseMrad", "ballDropProbability", "robotDropProbability");

    /** Field name -> "description [unit]". */
    static final Map<String, String> DOC = new LinkedHashMap<>();
    static {
        DOC.put("count", "Number of cameras of the automatic layout (null = 2 for division A, 1 for division B) [-]");
        DOC.put("height", "Default mounting height above the field [m]");
        DOC.put("focalLengthPx", "Focal length [px]; 0 = chosen so that a camera just covers its area");
        DOC.put("imageWidth", "Image width [px]");
        DOC.put("imageHeight", "Image height [px]");
        DOC.put("frequencyHz", "Capture rate [Hz]");
        DOC.put("positionNoiseMm", "Robot position noise (standard deviation) [mm]");
        DOC.put("ballNoiseMm", "Ball position noise (standard deviation) [mm]; negative = same as robots");
        DOC.put("orientationNoiseMrad", "Robot orientation noise (standard deviation) [mrad]");
        DOC.put("ballDropProbability", "Probability that the ball is missing in a frame [-]");
        DOC.put("robotDropProbability", "Probability that a robot is missing in a frame [-]");
        DOC.put("latencySeconds", "Delay between capture and publication of a frame [s]");
        DOC.put("overlapMeters", "Extra field area around a camera's area that it still reports [m]");
        DOC.put("positions", "Camera placement, one entry per camera, looking straight down (delete for the automatic layout):\n"
                + "#   x, y        position above the field, x along the field length, origin at the centre [m]\n"
                + "#   height      mounting height [m]; 0 = default height above\n"
                + "#   coverHalfX, coverHalfY  half extents of the field area the camera reports, centred below it [m]; 0 = automatic");
    }

    /** Default file with the camera positions of the automatic layout for {@code field}. */
    public static String defaultYaml(FieldGeometry field) {
        SimConfig.CameraConfig c = new SimConfig.CameraConfig();
        c.positions = VisionSystem.autoLayout(field, c);
        return toYaml(c);
    }

    public static String toYaml(SimConfig.CameraConfig c) {
        return ParamsYaml.toYaml(c, DOC, EXCLUDED,
                "# Camera parameters of the simulator (loaded after config.json; entries may be deleted to use the defaults).\n"
                + "# Noise, drop-out rate, latency and capture rate come from the realism preset and are not listed here.\n"
                + "# Units are SI unless stated.\n");
    }

    public static void apply(String yaml, SimConfig.CameraConfig target) {
        ParamsYaml.apply(yaml, target, SimConfig.CameraConfig.class, "camera params");
    }
}
