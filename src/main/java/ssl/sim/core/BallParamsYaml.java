package ssl.sim.core;

import ssl.sim.ball.Ball;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

/** YAML representation of {@link Ball.Params}: commented default file and loader (SnakeYAML). */
public final class BallParamsYaml {
    private BallParamsYaml() {}

    /** Field name -> "description [unit]". */
    static final Map<String, String> DOC = new LinkedHashMap<>();
    static {
        DOC.put("radius", "Ball radius [m] (golf ball used in SSL: 21.5 mm)");
        DOC.put("mass", "Ball mass [kg]");
        DOC.put("inertiaFactor", "Moment of inertia factor, I = factor * m * r^2 (0.4 solid sphere, 0.667 thin shell) [-]");
        DOC.put("muSlide", "Sliding friction coefficient between ball and ground [-]");
        DOC.put("aRoll", "Rolling deceleration of the ball on the ground [m/s^2]");
        DOC.put("spinFriction", "Decay rate of the ball spin about the vertical axis while touching the ground [1/s]");
        DOC.put("restitutionGround", "Coefficient of restitution of the vertical bounce on the ground [-]");
        DOC.put("muImpact", "Friction coefficient during ground impact (bounce) [-]");
        DOC.put("airDrag", "Quadratic air drag coefficient, force = c * |v| * v, only while airborne [kg/m]; 0 = none");
        DOC.put("muWall", "Ball-wall friction coefficient [-]; kept small so that a dribbler can peel the ball off a wall");
    }

    public static String defaultYaml() { return toYaml(new Ball.Params()); }

    public static String toYaml(Ball.Params p) {
        return ParamsYaml.toYaml(p, DOC, Set.of(),
                "# Ball parameters of the simulator (loaded after config.json; entries may be deleted to use the defaults).\n"
                + "# Units are SI unless stated.\n");
    }

    public static void apply(String yaml, Ball.Params target) {
        ParamsYaml.apply(yaml, target, Ball.Params.class, "ball params");
    }
}
