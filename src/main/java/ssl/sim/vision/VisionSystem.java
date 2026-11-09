package ssl.sim.vision;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import ssl.sim.core.FieldGeometry;
import ssl.sim.core.SimConfig;
import ssl.sim.core.World;

import java.util.ArrayList;
import java.util.List;

/** Set of cameras capturing at a fixed rate (default 60 Hz, simulated time). */
public class VisionSystem {
    private final List<Camera> cameras = new ArrayList<>();
    private final double period;
    private double nextCapture;
    /** Captured frames waiting for their delivery time (capture time + latency), in capture order. */
    private final java.util.ArrayDeque<Camera.DetectionFrame> pending = new java.util.ArrayDeque<>();

    public VisionSystem(double frequencyHz) { this.period = 1.0 / frequencyHz; }

    public static VisionSystem forField(FieldGeometry f) { return forField(f, new SimConfig.CameraConfig()); }

    /**
     * Cameras are mounted above the field centre line, tiled along the field length; each is responsible for an
     * equal strip (plus overlap). Defaults: 2 cameras for division A, 1 for division B, 6 m high.
     */
    public static VisionSystem forField(FieldGeometry f, SimConfig.CameraConfig cfg) {
        VisionSystem vs = new VisionSystem(cfg.frequencyHz);
        int n = cfg.effectiveCount(f.division);
        double totalX = f.length + 2 * f.boundaryWidth, totalY = f.width + 2 * f.boundaryWidth;
        double strip = totalX / n;
        for (int i = 0; i < n; i++) {
            double cx = -totalX / 2 + strip * (i + 0.5);
            double focal = cfg.focalLengthPx;
            if (focal <= 0) {
                double halfX = strip / 2 + cfg.overlapMeters, halfY = totalY / 2 + cfg.overlapMeters;
                focal = cfg.height * Math.min(cfg.imageWidth / 2.0 / halfX, cfg.imageHeight / 2.0 / halfY) / 1.02;
            }
            Camera c = new Camera(i, new Vector3D(cx, 0, cfg.height), focal, cfg.imageWidth, cfg.imageHeight,
                    cx, 0, strip / 2, totalY / 2);
            c.positionNoiseMm = cfg.positionNoiseMm;
            c.orientationNoise = cfg.orientationNoiseMrad / 1000.0;
            c.ballNoiseMm = cfg.ballNoiseMm;
            c.ballDropProbability = cfg.ballDropProbability;
            c.robotDropProbability = cfg.robotDropProbability;
            c.latency = cfg.latencySeconds;
            c.overlap = cfg.overlapMeters;
            vs.cameras.add(c);
        }
        return vs;
    }

    public List<Camera> cameras() { return cameras; }

    /**
     * Capture frames that are due at the given simulated time and return the frames that are delivered now,
     * i.e. whose capture time + camera latency ({@code tSent}) has been reached. The latency is therefore a real
     * delay between the physical state and the moment its detection is published.
     */
    public List<Camera.DetectionFrame> tick(World world) {
        while (world.time() >= nextCapture) {
            for (Camera c : cameras) pending.add(c.capture(world, nextCapture));
            nextCapture += period;
        }
        List<Camera.DetectionFrame> out = new ArrayList<>();
        while (!pending.isEmpty() && pending.peekFirst().tSent() <= world.time()) out.add(pending.pollFirst());
        return out;
    }

    public void resetClock(double time) { nextCapture = time; }
}
