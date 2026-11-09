package ssl.sim.core;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import ssl.sim.physics.ConvexPolygon;

import java.util.ArrayList;
import java.util.List;

/** Field dimensions from the RoboCup SSL rules (all lengths in metres). Blue defends -x, yellow +x by default. */
public final class FieldGeometry {
    public enum Division {
        A(12.0, 9.0, 1.8, 1.8, 3.6, 11),
        B(9.0, 6.0, 1.0, 1.0, 2.0, 8);

        final double length, width, goalWidth, penaltyDepth, penaltyWidth;
        public final int maxRobots;

        Division(double l, double w, double gw, double pd, double pw, int robots) {
            length = l;
            width = w;
            goalWidth = gw;
            penaltyDepth = pd;
            penaltyWidth = pw;
            maxRobots = robots;
        }
    }

    public final Division division;
    public final double length, width, goalWidth, goalDepth = 0.18, boundaryWidth = 0.3;
    public final double penaltyDepth, penaltyWidth, centerCircleRadius = 0.5, lineThickness = 0.01;
    public final double wallThickness = 0.02, wallHeight = 0.16;
    private final List<ConvexPolygon> walls = new ArrayList<>();
    private final List<ConvexPolygon> collisionWalls = new ArrayList<>();

    public FieldGeometry(Division d) { this(d, d.length, d.width, d.goalWidth, d.penaltyDepth, d.penaltyWidth); }

    public FieldGeometry(Division d, double length, double width, double goalWidth, double penaltyDepth, double penaltyWidth) {
        this.division = d;
        this.length = length;
        this.width = width;
        this.goalWidth = goalWidth;
        this.penaltyDepth = penaltyDepth;
        this.penaltyWidth = penaltyWidth;
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

    private void buildWalls() {
        buildCollisionWalls();
        double hl = halfLength(), hw = halfWidth(), b = boundaryWidth, t = wallThickness;
        // outer boundary walls
        walls.add(ConvexPolygon.box(0, hw + b + t / 2, hl + b + t, t / 2));
        walls.add(ConvexPolygon.box(0, -hw - b - t / 2, hl + b + t, t / 2));
        walls.add(ConvexPolygon.box(hl + b + t / 2, 0, t / 2, hw + b));
        walls.add(ConvexPolygon.box(-hl - b - t / 2, 0, t / 2, hw + b));
        // goals (back + two sides), on both ends
        for (int s = -1; s <= 1; s += 2) {
            double xBack = s * (hl + goalDepth + t / 2);
            walls.add(ConvexPolygon.box(xBack, 0, t / 2, goalWidth / 2 + t));
            double xMid = s * (hl + goalDepth / 2);
            walls.add(ConvexPolygon.box(xMid, goalWidth / 2 + t / 2, goalDepth / 2, t / 2));
            walls.add(ConvexPolygon.box(xMid, -goalWidth / 2 - t / 2, goalDepth / 2, t / 2));
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
        double hl = halfLength(), hw = halfWidth(), b = boundaryWidth, big = 1.0;
        // outer boundary: inner face at +-(half + b), 1 m thick outwards
        collisionWalls.add(ConvexPolygon.box(0, hw + b + big / 2, hl + b + big, big / 2));
        collisionWalls.add(ConvexPolygon.box(0, -hw - b - big / 2, hl + b + big, big / 2));
        collisionWalls.add(ConvexPolygon.box(hl + b + big / 2, 0, big / 2, hw + b + big));
        collisionWalls.add(ConvexPolygon.box(-hl - b - big / 2, 0, big / 2, hw + b + big));
        double th = 0.1; // goal walls: inner faces as in the visible model, extended outwards
        for (int s = -1; s <= 1; s += 2) {
            collisionWalls.add(ConvexPolygon.box(s * (hl + goalDepth + th / 2), 0, th / 2, goalWidth / 2 + th));
            double xMid = s * (hl + goalDepth / 2);
            collisionWalls.add(ConvexPolygon.box(xMid, goalWidth / 2 + th / 2, goalDepth / 2, th / 2));
            collisionWalls.add(ConvexPolygon.box(xMid, -goalWidth / 2 - th / 2, goalDepth / 2, th / 2));
        }
    }
}
