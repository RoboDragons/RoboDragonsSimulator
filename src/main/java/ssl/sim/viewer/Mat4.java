package ssl.sim.viewer;

/** Minimal column-major 4x4 float matrix helpers for the viewer. */
final class Mat4 {
    private Mat4() {}

    static float[] identity() {
        return new float[]{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    }

    static float[] mul(float[] a, float[] b) {
        float[] r = new float[16];
        for (int c = 0; c < 4; c++)
            for (int rr = 0; rr < 4; rr++) {
                float s = 0;
                for (int k = 0; k < 4; k++) s += a[k * 4 + rr] * b[c * 4 + k];
                r[c * 4 + rr] = s;
            }
        return r;
    }

    static float[] translate(double x, double y, double z) {
        float[] m = identity();
        m[12] = (float) x;
        m[13] = (float) y;
        m[14] = (float) z;
        return m;
    }

    static float[] scale(double x, double y, double z) {
        float[] m = identity();
        m[0] = (float) x;
        m[5] = (float) y;
        m[10] = (float) z;
        return m;
    }

    static float[] rotZ(double a) {
        float c = (float) Math.cos(a), s = (float) Math.sin(a);
        float[] m = identity();
        m[0] = c;
        m[1] = s;
        m[4] = -s;
        m[5] = c;
        return m;
    }

    static float[] rotY(double a) {
        float c = (float) Math.cos(a), s = (float) Math.sin(a);
        float[] m = identity();
        m[0] = c;
        m[2] = -s;
        m[8] = s;
        m[10] = c;
        return m;
    }

    static float[] rotX(double a) {
        float c = (float) Math.cos(a), s = (float) Math.sin(a);
        float[] m = identity();
        m[5] = c;
        m[6] = s;
        m[9] = -s;
        m[10] = c;
        return m;
    }

    static float[] perspective(double fovY, double aspect, double near, double far) {
        double f = 1.0 / Math.tan(fovY / 2);
        float[] m = new float[16];
        m[0] = (float) (f / aspect);
        m[5] = (float) f;
        m[10] = (float) ((far + near) / (near - far));
        m[11] = -1;
        m[14] = (float) (2 * far * near / (near - far));
        return m;
    }

    static float[] lookAt(double ex, double ey, double ez, double cx, double cy, double cz, double ux, double uy, double uz) {
        double fx = cx - ex, fy = cy - ey, fz = cz - ez;
        double fl = Math.sqrt(fx * fx + fy * fy + fz * fz);
        fx /= fl;
        fy /= fl;
        fz /= fl;
        double sx = fy * uz - fz * uy, sy = fz * ux - fx * uz, sz = fx * uy - fy * ux;
        double sl = Math.sqrt(sx * sx + sy * sy + sz * sz);
        sx /= sl;
        sy /= sl;
        sz /= sl;
        double vx = sy * fz - sz * fy, vy = sz * fx - sx * fz, vz = sx * fy - sy * fx;
        float[] m = identity();
        m[0] = (float) sx;
        m[4] = (float) sy;
        m[8] = (float) sz;
        m[1] = (float) vx;
        m[5] = (float) vy;
        m[9] = (float) vz;
        m[2] = (float) -fx;
        m[6] = (float) -fy;
        m[10] = (float) -fz;
        m[12] = (float) -(sx * ex + sy * ey + sz * ez);
        m[13] = (float) -(vx * ex + vy * ey + vz * ez);
        m[14] = (float) (fx * ex + fy * ey + fz * ez);
        return m;
    }
}
