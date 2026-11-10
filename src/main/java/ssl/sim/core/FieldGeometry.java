package ssl.sim.core;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import ssl.sim.physics.ConvexPolygon;

import java.util.ArrayList;
import java.util.List;

/** Field dimensions (all lengths in metres, see {@link FieldParams}). Blue defends -x, yellow +x by default. */
public final class FieldGeometry {
    public enum Division {
        A(11), B(8);

        public final int maxRobots;

        Division(int robots) { maxRobots = robots; }
    }

    public final Division division;
    public final double length, width, goalWidth, goalDepth;
    /** Margin to the outer wall along the touch lines (y direction) and behind the goal lines (x direction). */
    public final double boundaryTouchLine, boundaryGoalLine;
    public final boolean goalSideWallsToBoundary;
    public final double penaltyDepth, penaltyWidth, centerCircleRadius, lineThickness;
    public final double wallThickness, wallHeight;
    private final List<ConvexPolygon> walls = new ArrayList<>();
    private final List<ConvexPolygon> collisionWalls = new ArrayList<>();

    public FieldGeometry(Division d) { this(d, FieldParams.defaultsFor(d)); }

    /** Division defaults with the main dimensions replaced (used by the simulation protocol). */
    public FieldGeometry(Division d, double length, double width, double goalWidth, double penaltyDepth, double penaltyWidth) {
        this(d, withMainDimensions(d, length, width, goalWidth, penaltyDepth, penaltyWidth));
    }

    private static FieldParams withMainDimensions(Division d, double l, double w, double gw, double pd, double pw) {
        FieldParams p = FieldParams.defaultsFor(d);
        p.length = l;
        p.width = w;
        p.goalWidth = gw;
        p.penaltyDepth = pd;
        p.penaltyWidth = pw;
        return p;
    }

    public FieldGeometry(Division d, FieldParams p) {
        this.division = d;
        this.length = p.length;
        this.width = p.width;
        this.goalWidth = p.goalWidth;
        this.goalDepth = p.goalDepth;
        this.boundaryTouchLine = p.boundaryTouchLine;
        this.boundaryGoalLine = p.boundaryGoalLine;
        this.goalSideWallsToBoundary = p.goalSideWallsToBoundary;
        this.penaltyDepth = p.penaltyDepth;
        this.penaltyWidth = p.penaltyWidth;
        this.centerCircleRadius = p.centerCircleRadius;
        this.lineThickness = p.lineThickness;
        this.wallThickness = p.wallThickness;
        this.wallHeight = p.wallHeight;
        buildWalls();
    }

    public double halfLength() { return length / 2; }
    public double halfWidth() { return width / 2; }

    /** Thin visible walls (for rendering). */
    public List<ConvexPolygon> walls() { return walls; }

    /**
     * Walls used for collision: same inner faces as {@link #walls()} but extended outward so that they are
     * thick. A thin wall can be tunnelled by the penalty contact once a fast body penetrates deeper than the
     * wall thickness (the contact normal then flips); thick walls keep the normal pointing into the field.
     */
    public List<ConvexPolygon> collisionWalls() { return collisionWalls; }

    /** Length of the goal side walls along x, starting at the goal line: the goal depth, or up to the outer wall. */
    private double goalSideWallLength() { return goalSideWallsToBoundary ? Math.max(goalDepth, boundaryGoalLine) : goalDepth; }

    private void buildWalls() {
        buildCollisionWalls();
        double hl = halfLength(), hw = halfWidth(), bx = boundaryGoalLine, by = boundaryTouchLine, t = wallThickness;
        // outer boundary walls
        walls.add(ConvexPolygon.box(0, hw + by + t / 2, hl + bx + t, t / 2));
        walls.add(ConvexPolygon.box(0, -hw - by - t / 2, hl + bx + t, t / 2));
        walls.add(ConvexPolygon.box(hl + bx + t / 2, 0, t / 2, hw + by));
        walls.add(ConvexPolygon.box(-hl - bx - t / 2, 0, t / 2, hw + by));
        // goals (back + two sides), on both ends
        double sideLen = goalSideWallLength();
        for (int s = -1; s <= 1; s += 2) {
            double xBack = s * (hl + goalDepth + t / 2);
            walls.add(ConvexPolygon.box(xBack, 0, t / 2, goalWidth / 2 + t));
            double xMid = s * (hl + sideLen / 2);
            walls.add(ConvexPolygon.box(xMid, goalWidth / 2 + t / 2, sideLen / 2, t / 2));
            walls.add(ConvexPolygon.box(xMid, -goalWidth / 2 - t / 2, sideLen / 2, t / 2));
        }
    }

    public boolean isInsideField(Vector2D p) {
        return Math.abs(p.getX()) <= halfLength() && Math.abs(p.getY()) <= halfWidth();
    }

    /** Which goal the point is in: +1 = at +x goal, -1 = at -x goal, 0 = none. */
    public int goalAt(double x, double y) {
        if (Math.abs(y) < goalWidth / 2 && Math.abs(x) > halfLength() && Math.abs(x) < halfLength() + goalDepth + 0.05) {
            return x > 0 ? 1 : -1;
        }
        return 0;
    }

    public boolean inPenaltyArea(Vector2D p, int side) {
        return side * p.getX() >= halfLength() - penaltyDepth && side * p.getX() <= halfLength()
                && Math.abs(p.getY()) <= penaltyWidth / 2;
    }

    private void buildCollisionWalls() {
        double hl = halfLength(), hw = halfWidth(), bx = boundaryGoalLine, by = boundaryTouchLine, big = 1.0;
        // outer boundary: inner face at +-(half + margin), 1 m thick outwards
        collisionWalls.add(ConvexPolygon.box(0, hw + by + big / 2, hl + bx + big, big / 2));
        collisionWalls.add(ConvexPolygon.box(0, -hw - by - big / 2, hl + bx + big, big / 2));
        collisionWalls.add(ConvexPolygon.box(hl + bx + big / 2, 0, big / 2, hw + by + big));
        collisionWalls.add(ConvexPolygon.box(-hl - bx - big / 2, 0, big / 2, hw + by + big));
        double th = 0.1; // goal walls: inner faces as in the visible model, extended outwards
        double sideLen = goalSideWallLength();
        for (int s = -1; s <= 1; s += 2) {
            collisionWalls.add(ConvexPolygon.box(s * (hl + goalDepth + th / 2), 0, th / 2, goalWidth / 2 + th));
            double xMid = s * (hl + sideLen / 2);
            collisionWalls.add(ConvexPolygon.box(xMid, goalWidth / 2 + th / 2, sideLen / 2, th / 2));
            collisionWalls.add(ConvexPolygon.box(xMid, -goalWidth / 2 - th / 2, sideLen / 2, th / 2));
        }
    }
}
