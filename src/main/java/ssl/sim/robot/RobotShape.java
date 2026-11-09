package ssl.sim.robot;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import ssl.sim.physics.ConvexPolygon;

import java.util.ArrayList;
import java.util.List;

/**
 * Robot outline: a circle of radius r cut flat at {@code centerToDribbler} (SSL convex-hull rule).
 * For ball contacts the hull is split into convex pieces so that a notch for the dribbler roller remains:
 * the ball may sit up to ~20 % of its diameter inside the convex hull, as allowed by the rules.
 */
public final class RobotShape {
    private static final int ARC_SEGMENTS = 24;
    private final ConvexPolygon hull;
    private final List<ConvexPolygon> ballPieces = new ArrayList<>();

    public RobotShape(RobotSpecs s) {
        double xf = s.centerToDribbler, r = s.radius;
        double a0 = Math.acos(xf / r);
        Vector2D[] pts = new Vector2D[ARC_SEGMENTS + 1];
        for (int i = 0; i <= ARC_SEGMENTS; i++) {
            double a = a0 + i * (2 * Math.PI - 2 * a0) / ARC_SEGMENTS;
            pts[i] = new Vector2D(r * Math.cos(a), r * Math.sin(a));
        }
        pts[0] = new Vector2D(xf, r * Math.sin(a0));
        pts[ARC_SEGMENTS] = new Vector2D(xf, -r * Math.sin(a0));
        hull = new ConvexPolygon(pts);

        double xFloor = s.notchFloorX();
        double nh = s.notchHalfWidth;
        ballPieces.add(hull.clip(new Vector2D(xFloor, 0), new Vector2D(-1, 0)));
        ConvexPolygon front = hull.clip(new Vector2D(xFloor, 0), new Vector2D(1, 0));
        ballPieces.add(front.clip(new Vector2D(0, -nh), new Vector2D(0, -1)));
        ballPieces.add(front.clip(new Vector2D(0, nh), new Vector2D(0, 1)));
    }

    public ConvexPolygon hull() { return hull; }
    public List<ConvexPolygon> ballPieces() { return ballPieces; }
}
