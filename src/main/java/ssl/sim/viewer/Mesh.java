package ssl.sim.viewer;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.util.ArrayList;
import java.util.List;

import static org.lwjgl.opengl.GL33C.*;

/** Static indexed triangle mesh with interleaved position + normal, uploaded once to the GPU. */
final class Mesh implements AutoCloseable {
    private final int vao, vbo, ebo, count;

    private Mesh(float[] vertices, int[] indices) {
        count = indices.length;
        vao = glGenVertexArrays();
        vbo = glGenBuffers();
        ebo = glGenBuffers();
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, vertices, GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, false, 24, 0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, false, 24, 12);
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
    }

    void draw() {
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, 0);
    }

    @Override
    public void close() {
        glDeleteBuffers(vbo);
        glDeleteBuffers(ebo);
        glDeleteVertexArrays(vao);
    }

    private static final class Builder {
        final List<Float> v = new ArrayList<>();
        final List<Integer> idx = new ArrayList<>();

        int vertex(double x, double y, double z, double nx, double ny, double nz) {
            v.add((float) x);
            v.add((float) y);
            v.add((float) z);
            v.add((float) nx);
            v.add((float) ny);
            v.add((float) nz);
            return v.size() / 6 - 1;
        }

        void tri(int a, int b, int c) {
            idx.add(a);
            idx.add(b);
            idx.add(c);
        }

        void quad(int a, int b, int c, int d) {
            tri(a, b, c);
            tri(a, c, d);
        }

        Mesh build() {
            float[] vf = new float[v.size()];
            for (int i = 0; i < vf.length; i++) vf[i] = v.get(i);
            return new Mesh(vf, idx.stream().mapToInt(Integer::intValue).toArray());
        }
    }

    /** Unit cube centred at the origin. */
    static Mesh cube() {
        Builder b = new Builder();
        double[][] n = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
        for (double[] nn : n) {
            double[] u = nn[0] != 0 ? new double[]{0, 1, 0} : nn[1] != 0 ? new double[]{0, 0, 1} : new double[]{1, 0, 0};
            double[] w = {nn[1] * u[2] - nn[2] * u[1], nn[2] * u[0] - nn[0] * u[2], nn[0] * u[1] - nn[1] * u[0]};
            int[] i = new int[4];
            int k = 0;
            for (int su = -1; su <= 1; su += 2)
                for (int sw = -1; sw <= 1; sw += 2) {
                    i[k++] = b.vertex(0.5 * nn[0] + 0.5 * su * u[0] + 0.5 * sw * w[0],
                            0.5 * nn[1] + 0.5 * su * u[1] + 0.5 * sw * w[1],
                            0.5 * nn[2] + 0.5 * su * u[2] + 0.5 * sw * w[2], nn[0], nn[1], nn[2]);
                }
            // winding is fixed up by disabling face culling in the viewer
            b.quad(i[0], i[1], i[3], i[2]);
        }
        return b.build();
    }

    /** Cylinder of radius 1 along z, from -0.5 to +0.5. */
    static Mesh cylinder(int segments) {
        Builder b = new Builder();
        for (int i = 0; i <= segments; i++) {
            double a = 2 * Math.PI * i / segments, c = Math.cos(a), s = Math.sin(a);
            b.vertex(c, s, -0.5, c, s, 0);
            b.vertex(c, s, 0.5, c, s, 0);
        }
        for (int i = 0; i < segments; i++) b.quad(2 * i, 2 * i + 1, 2 * i + 3, 2 * i + 2);
        for (int cap = 0; cap < 2; cap++) {
            double z = cap == 0 ? -0.5 : 0.5, nz = cap == 0 ? -1 : 1;
            int center = b.vertex(0, 0, z, 0, 0, nz);
            int first = b.v.size() / 6;
            for (int i = 0; i <= segments; i++) {
                double a = 2 * Math.PI * i / segments;
                b.vertex(Math.cos(a), Math.sin(a), z, 0, 0, nz);
            }
            for (int i = 0; i < segments; i++) b.tri(center, first + i, first + i + 1);
        }
        return b.build();
    }

    /** Unit sphere. */
    static Mesh sphere(int stacks, int slices) {
        Builder b = new Builder();
        for (int i = 0; i <= stacks; i++) {
            double phi = Math.PI * i / stacks;
            for (int j = 0; j <= slices; j++) {
                double th = 2 * Math.PI * j / slices;
                double x = Math.sin(phi) * Math.cos(th), y = Math.sin(phi) * Math.sin(th), z = Math.cos(phi);
                b.vertex(x, y, z, x, y, z);
            }
        }
        for (int i = 0; i < stacks; i++)
            for (int j = 0; j < slices; j++) {
                int a = i * (slices + 1) + j, c = a + slices + 1;
                b.quad(a, c, c + 1, a + 1);
            }
        return b.build();
    }

    /** Convex polygon (CCW) extruded from z0 to z1. */
    static Mesh prism(Vector2D[] poly, double z0, double z1) {
        Builder b = new Builder();
        int n = poly.length;
        for (int i = 0; i < n; i++) {
            Vector2D p = poly[i], q = poly[(i + 1) % n];
            double ex = q.getX() - p.getX(), ey = q.getY() - p.getY(), len = Math.hypot(ex, ey);
            double nx = ey / len, ny = -ex / len;
            int a = b.vertex(p.getX(), p.getY(), z0, nx, ny, 0), c = b.vertex(q.getX(), q.getY(), z0, nx, ny, 0);
            int d = b.vertex(q.getX(), q.getY(), z1, nx, ny, 0), e = b.vertex(p.getX(), p.getY(), z1, nx, ny, 0);
            b.quad(a, c, d, e);
        }
        double cx = 0, cy = 0;
        for (Vector2D p : poly) {
            cx += p.getX() / n;
            cy += p.getY() / n;
        }
        int center = b.vertex(cx, cy, z1, 0, 0, 1);
        int first = b.v.size() / 6;
        for (Vector2D p : poly) b.vertex(p.getX(), p.getY(), z1, 0, 0, 1);
        for (int i = 0; i < n; i++) b.tri(center, first + i, first + (i + 1) % n);
        return b.build();
    }
}
