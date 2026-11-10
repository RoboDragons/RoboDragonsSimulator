package ssl.sim.core;

/**
 * Field parameters in metres (YAML: field_params.yaml). The defaults follow the 2026 SSL rules: Division A has a
 * 0.3 m margin along the touch lines and 0.6 m behind the goal lines (the extra 0.3 m is the Goal Substitution Area), and
 * the goal side walls continue to the outer wall so that the area behind each goal is partitioned off.
 */
public class FieldParams {
    public double length, width;
    public double goalWidth, goalDepth = 0.18;
    public double penaltyDepth, penaltyWidth;
    /** Margin between the touch lines and the outer wall. */
    public double boundaryTouchLine = 0.3;
    /** Margin between the goal lines and the outer wall. */
    public double boundaryGoalLine = 0.3;
    /** Goal side walls continue from the goal back wall to the outer wall (partitions the substitution area). */
    public boolean goalSideWallsToBoundary = false;
    public double centerCircleRadius = 0.5;
    public double lineThickness = 0.01;
    public double wallThickness = 0.02;
    public double wallHeight = 0.16;

    public static FieldParams defaultsFor(FieldGeometry.Division d) {
        FieldParams p = new FieldParams();
        switch (d) {
            case A -> {
                p.length = 12.0;
                p.width = 9.0;
                p.goalWidth = 1.8;
                p.penaltyDepth = 1.8;
                p.penaltyWidth = 3.6;
                p.boundaryGoalLine = 0.6;
                p.goalSideWallsToBoundary = true;
            }
            case B -> {
                p.length = 9.0;
                p.width = 6.0;
                p.goalWidth = 1.0;
                p.penaltyDepth = 1.0;
                p.penaltyWidth = 2.0;
            }
        }
        return p;
    }
}
