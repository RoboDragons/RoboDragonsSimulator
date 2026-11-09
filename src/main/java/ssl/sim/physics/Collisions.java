package ssl.sim.physics;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

/** 2D narrow-phase collision routines. */
public final class Collisions {
    private Collisions() {}

    /** Contact result. {@code normal} points from the first shape to the second one. */
    public record Contact(Vector2D normal, double depth, Vector2D point) {}

    /** Circle vs. convex polygon. Normal points from the polygon towards the circle. */
    public static Contact circleVsPolygon(Vector2D c, double r, ConvexPolygon poly) {
        Vector2D[] v = poly.vertices();
        int n = v.length;
        double best = -Double.MAX_VALUE;
        int bestEdge = 0;
        for (int i = 0; i < n; i++) {
            double s = poly.edgeNormal(i).dotProduct(c.subtract(v[i]));
            if (s > best) {
                best = s;
                bestEdge = i;
            }
        }
        if (best > r) return null;
        if (best <= 0) { // centre inside
            Vector2D nrm = poly.edgeNormal(bestEdge);
            return new Contact(nrm, r - best, c.subtract(nrm.scalarMultiply(best)));
        }
        Vector2D closest = null;
        double bestD = Double.MAX_VALUE;
        for (int i = 0; i < n; i++) {
            Vector2D a = v[i], b = v[(i + 1) % n];
            Vector2D ab = b.subtract(a);
            double t = Math.max(0, Math.min(1, c.subtract(a).dotProduct(ab) / ab.getNormSq()));
            Vector2D q = a.add(ab.scalarMultiply(t));
            double d = c.distance(q);
            if (d < bestD) {
                bestD = d;
                closest = q;
            }
        }
        if (bestD >= r || bestD < 1e-12) return null;
        Vector2D nrm = c.subtract(closest).scalarMultiply(1.0 / bestD);
        return new Contact(nrm, r - bestD, closest);
    }

    /**
     * Separating axis test for two convex polygons. Normal points from a to b. The contact point is taken on the
     * incident polygon (the one whose vertex penetrates the other's reference face); if its face is parallel to the
     * reference face the middle of the touching edge is used, so large flat faces (walls) never produce a contact
     * point far away from the actual contact, which would create huge spurious torques.
     */
    public static Contact polygonVsPolygon(ConvexPolygon a, ConvexPolygon b) {
        double minOverlap = Double.MAX_VALUE;
        Vector2D bestAxis = null;
        boolean ownerIsA = true;
        for (int pass = 0; pass < 2; pass++) {
            ConvexPolygon p = pass == 0 ? a : b;
            for (int i = 0; i < p.size(); i++) {
                Vector2D axis = p.edgeNormal(i);
                double[] ra = project(a, axis), rb = project(b, axis);
                double overlap = Math.min(ra[1], rb[1]) - Math.max(ra[0], rb[0]);
                if (overlap <= 0) return null;
                if (overlap < minOverlap) {
                    minOverlap = overlap;
                    bestAxis = axis;
                    ownerIsA = pass == 0;
                }
            }
        }
        if (bestAxis.dotProduct(b.centroid().subtract(a.centroid())) < 0) bestAxis = bestAxis.negate();
        Vector2D point;
        if (ownerIsA) { // b's vertex (or edge) penetrates a's face
            point = support(b, bestAxis.negate()).add(bestAxis.scalarMultiply(minOverlap / 2));
        } else {        // a's vertex (or edge) penetrates b's face
            point = support(a, bestAxis).subtract(bestAxis.scalarMultiply(minOverlap / 2));
        }
        return new Contact(bestAxis, minOverlap, point);
    }

    private static double[] project(ConvexPolygon p, Vector2D axis) {
        double mn = Double.MAX_VALUE, mx = -Double.MAX_VALUE;
        for (Vector2D q : p.vertices()) {
            double d = q.dotProduct(axis);
            mn = Math.min(mn, d);
            mx = Math.max(mx, d);
        }
        return new double[]{mn, mx};
    }

    /** Extreme point of the polygon along dir; vertices tied for the extreme are averaged. */
    private static Vector2D support(ConvexPolygon p, Vector2D dir) {
        double bd = -Double.MAX_VALUE;
        for (Vector2D q : p.vertices()) bd = Math.max(bd, q.dotProduct(dir));
        double sx = 0, sy = 0;
        int n = 0;
        for (Vector2D q : p.vertices()) {
            if (q.dotProduct(dir) >= bd - 1e-6) {
                sx += q.getX();
                sy += q.getY();
                n++;
            }
        }
        return new Vector2D(sx / n, sy / n);
    }
}
