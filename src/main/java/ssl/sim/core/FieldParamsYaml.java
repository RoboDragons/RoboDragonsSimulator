package ssl.sim.core;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

/** YAML representation of {@link FieldParams}: commented default file for a division and loader (SnakeYAML). */
public final class FieldParamsYaml {
    private FieldParamsYaml() {}

    /** Field name -> "description [unit]". */
    static final Map<String, String> DOC = new LinkedHashMap<>();
    static {
        DOC.put("length", "Playing area length between the goal lines (x direction) [m]");
        DOC.put("width", "Playing area width between the touch lines (y direction) [m]");
        DOC.put("goalWidth", "Inner distance between the goal side walls [m]");
        DOC.put("goalDepth", "Goal depth from the goal line to the inner face of the back wall [m]");
        DOC.put("penaltyDepth", "Defense area depth from the goal line [m]");
        DOC.put("penaltyWidth", "Defense area width [m]");
        DOC.put("boundaryTouchLine", "Margin between the touch lines and the outer wall [m] (rules: 0.3)");
        DOC.put("boundaryGoalLine", "Margin between the goal lines and the outer wall [m] (rules: Division A 0.6, the part beyond 0.3 is the Goal Substitution Area; Division B 0.3)");
        DOC.put("goalSideWallsToBoundary", "Goal side walls continue to the outer wall, partitioning the area behind the goal (Division A rules) [-]");
        DOC.put("centerCircleRadius", "Radius of the centre circle [m]");
        DOC.put("lineThickness", "Field line width [m]");
        DOC.put("wallThickness", "Wall thickness [m]");
        DOC.put("wallHeight", "Wall and goal wall height [m]");
    }

    /** Default file for the given division (a changed division needs a matching file or none). */
    public static String defaultYaml(FieldGeometry.Division d) { return toYaml(FieldParams.defaultsFor(d), d); }

    public static String toYaml(FieldParams p, FieldGeometry.Division d) {
        return ParamsYaml.toYaml(p, DOC, Set.of(),
                "# Field parameters of the simulator (loaded after config.json; entries may be deleted to use the defaults of the division).\n"
                + "# These are the defaults of Division " + d + " (2026 rules). Origin at the field centre, x along the length.\n"
                + "# --division selects the defaults of that division and ignores this file. Units are SI unless stated.\n");
    }

    public static void apply(String yaml, FieldParams target) {
        ParamsYaml.apply(yaml, target, FieldParams.class, "field params");
    }
}
