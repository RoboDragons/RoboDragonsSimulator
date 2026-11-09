package ssl.sim.vision;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import ssl.sim.core.World;
import ssl.sim.robot.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Pinhole camera looking at (part of) the field, like an SSL-Vision camera. It owns its calibration and
 * produces detection frames in vision coordinates (millimetres, pixels) with optional noise and dropouts.
 */
public class Camera {
    public record BallDetection(double confidence, double xMm, double yMm, double z, double pixelX, double pixelY) {}

    public record RobotDetection(double confidence, int id, double xMm, double yMm, double orientation,
                                 double pixelX, double pixelY, double height) {}

    public record DetectionFrame(int frameNumber, double tCapture, double tSent, int cameraId,
                                 List<BallDetection> balls, List<RobotDetection> yellow, List<RobotDetection> blue) {}

    public final int id;
    private final Vector3D position;           // camera centre in world [m]
    private final Rotation worldToCamera;
    private final double focalLength;          // [px]
    private final double principalX, principalY;
    private final int width, height;
    private final double distortion;
    // area of the field this camera is responsible for (centre + half extents), plus overlap
    private final double coverX, coverY, coverHalfX, coverHalfY;

    public double positionNoiseMm = 1.0;
    public double orientationNoise = 0.003;
    public double ballNoiseMm = -1;
    public double ballDropProbability = 0.0;
    public double robotDropProbability = 0.0;
    public double latency = 0.0;
    public double overlap = 0.25;
    private final Random random = new Random(id());
    private int frameNumber;

    public Camera(int id, Vector3D position, double focalLength, int width, int height,
                  double coverX, double coverY, double coverHalfX, double coverHalfY) {
        this.id = id;
        this.position = position;
        // camera looking straight down: camera x = world +x, camera y = world -y (image down), camera z = world -z
        this.worldToCamera = new Rotation(Vector3D.PLUS_I, Math.PI);
        this.focalLength = focalLength;
        this.width = width;
        this.height = height;
        this.principalX = width / 2.0;
        this.principalY = height / 2.0;
        this.distortion = 0.0;
        this.coverX = coverX;
        this.coverY = coverY;
        this.coverHalfX = coverHalfX;
        this.coverHalfY = coverHalfY;
    }

    private int id() { return id; }

    public Vector3D position() { return position; }
    public double focalLength() { return focalLength; }
    public int width() { return width; }
    public int height() { return height; }
    public double principalX() { return principalX; }
    public double principalY() { return principalY; }
    public double distortion() { return distortion; }
    public Rotation worldToCamera() { return worldToCamera; }

    /** Translation of the world->camera transform in millimetres: p_cam = R * p_world + t. */
    public Vector3D translationMm() {
        return worldToCamera.applyTo(position).negate().scalarMultiply(1000.0);
    }

    /** Project a world point [m] into pixel coordinates, or null if it is behind the camera / outside the image. */
    public double[] project(Vector3D world) {
        Vector3D pc = worldToCamera.applyTo(world.subtract(position));
        if (pc.getZ() <= 0.05) return null;
        double xn = pc.getX() / pc.getZ(), yn = pc.getY() / pc.getZ();
        double r2 = xn * xn + yn * yn;
        double k = 1 + distortion * r2;
        double px = principalX + focalLength * xn * k, py = principalY + focalLength * yn * k;
        if (px < 0 || px >= width || py < 0 || py >= height) return null;
        return new double[]{px, py};
    }

    private boolean inCoverage(double x, double y) {
        return Math.abs(x - coverX) <= coverHalfX + overlap && Math.abs(y - coverY) <= coverHalfY + overlap;
    }

    public DetectionFrame capture(World world, double time) {
        List<BallDetection> balls = new ArrayList<>();
        List<RobotDetection> yellow = new ArrayList<>(), blue = new ArrayList<>();
        var b = world.ball();
        if (inCoverage(b.x(), b.y()) && random.nextDouble() >= ballDropProbability) {
            double[] px = project(b.position());
            if (px != null) {
                double bn = ballNoiseMm >= 0 ? ballNoiseMm : positionNoiseMm;
                balls.add(new BallDetection(0.95, (b.x() * 1000) + noise(bn), (b.y() * 1000) + noise(bn),
                        0, px[0], px[1]));
            }
        }
        for (Robot r : world.robots()) {
            if (!inCoverage(r.x(), r.y()) || random.nextDouble() < robotDropProbability) continue;
            double[] px = project(new Vector3D(r.x(), r.y(), r.specs().height));
            if (px == null) continue;
            RobotDetection d = new RobotDetection(0.95, r.id, r.x() * 1000 + noise(positionNoiseMm),
                    r.y() * 1000 + noise(positionNoiseMm), r.heading() + noise(orientationNoise),
                    px[0], px[1], r.specs().height * 1000);
            (r.team == Robot.Team.YELLOW ? yellow : blue).add(d);
        }
        return new DetectionFrame(frameNumber++, time, time + latency, id, balls, yellow, blue);
    }

    private double noise(double sigma) { return sigma == 0 ? 0 : random.nextGaussian() * sigma; }

    /** Center of the covered area, for debugging / viewer. */
    public Vector2D coverageCenter() { return new Vector2D(coverX, coverY); }
}
