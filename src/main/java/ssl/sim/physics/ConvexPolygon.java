package ssl.sim.physics;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.util.ArrayList;
import java.util.List;

/** Counter-clockwise convex polygon in the plane (used for robot hulls, walls, ball-contact pieces). */
public final class ConvexPolygon {
    private final Vector2D[] v;

    public ConvexPolygon(Vector2D... vertices) {
        this.v = vertices.clone();
    }

    public static ConvexPolygon box(double cx, double cy, double hx, double hy) {
        return new ConvexPolygon(
                new Vector2D(cx - hx, cy - hy), new Vector2D(cx + hx, cy - hy),
                new Vector2D(cx + hx, cy + hy), new Vector2D(cx - hx, cy + hy));
    }

    public Vector2D[] vertices() {
        return v;
    }

    public int size() {
        return v.length;
    }

    public Vector2D centroid() {
        double sx = 0, sy = 0;
        for (Vector2D p : v) {
            sx += p.getX();
            sy += p.getY();
        }
        return new Vector2D(sx / v.length, sy / v.length);
    }

    /** Outward unit normal of edge i (from vertex i to i+1). */
    public Vector2D edgeNormal(int i) {
        Vector2D a = v[i], b = v[(i + 1) % v.length];
        double ex = b.getX() - a.getX(), ey = b.getY() - a.getY();
        double len = Math.hypot(ex, ey);
        return new Vector2D(ey / len, -ex / len);
    }

    public ConvexPolygon transform(double px, double py, double theta) {
        double c = Math.cos(theta), s = Math.sin(theta);
        Vector2D[] out = new Vector2D[v.length];
        for (int i = 0; i < v.length; i++) {
            double x = v[i].getX(), y = v[i].getY();
            out[i] = new Vector2D(px + c * x - s * y, py + s * x + c * y);
        }
        return new ConvexPolygon(out);
    }

    /** Sutherland-Hodgman clip: keeps the part where dot(p - point, normal) >= 0. */
    public ConvexPolygon clip(Vector2D point, Vector2D normal) {
        List<Vector2D> out = new ArrayList<>();
        for (int i = 0; i < v.length; i++) {
            Vector2D a = v[i], b = v[(i + 1) % v.length];
            double da = a.subtract(point).dotProduct(normal);
            double db = b.subtract(point).dotProduct(normal);
            if (da >= 0) out.add(a);
            if ((da >= 0) != (db >= 0)) {
                double t = da / (da - db);
                out.add(new Vector2D(a.getX() + t * (b.getX() - a.getX()), a.getY() + t * (b.getY() - a.getY())));
            }
        }
        return new ConvexPolygon(out.toArray(new Vector2D[0]));
    }
}
