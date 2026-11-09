package ssl.sim.core;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import ssl.sim.robot.Robot;
import ssl.sim.robot.RobotSpecs;
import ssl.sim.vision.Camera;
import ssl.sim.vision.VisionSystem;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Consumer;

/** Owns world, vision cameras, and steps them in real time (scaled) on its own thread. */
public class Simulator {
    private volatile World world;
    private volatile VisionSystem vision;
    private final Map<String, RobotSpecs> specOverrides = new HashMap<>();
    private final List<Consumer<Camera.DetectionFrame>> frameListeners = new CopyOnWriteArrayList<>();
    private volatile double speed = 1.0;
    private volatile boolean paused;
    private volatile boolean running;
    private Thread thread;

    private final SimConfig config;
    private volatile Telemetry telemetry;
    public Telemetry telemetry() { return telemetry; }
    public void setTelemetry(Telemetry t) { telemetry = t; }

    public Simulator(FieldGeometry field) { this(field, new SimConfig()); }

    public Simulator(SimConfig config) { this(config.field(), config); }

    public Simulator(FieldGeometry field, SimConfig config) {
        this.config = config;
        world = new World(field, config.ball);
        vision = VisionSystem.forField(field, config.cameras);
    }

    public SimConfig config() { return config; }

    public World world() { return world; }
    public VisionSystem vision() { return vision; }
    public double speed() { return speed; }
    public void setSpeed(double s) { speed = Math.max(0.0, Math.min(20.0, s)); }
    public boolean paused() { return paused; }
    public void setPaused(boolean p) { paused = p; }
    public void addFrameListener(Consumer<Camera.DetectionFrame> l) { frameListeners.add(l); }

    /** Replace the field (e.g. via simulator config); robots and ball are kept, cameras are re-laid out. */
    public void setField(FieldGeometry f) {
        world.lock().lock();
        try {
            world.setField(f);
            vision = VisionSystem.forField(f, config.cameras);
            vision.resetClock(world.time());
        } finally {
            world.lock().unlock();
        }
    }

    public RobotSpecs specsFor(Robot.Team team, int id) {
        RobotSpecs s = specOverrides.get(team + ":" + id);
        return s != null ? s.copy() : config.robot.copy();
    }

    public void setSpecs(Robot.Team team, int id, RobotSpecs s) { specOverrides.put(team + ":" + id, s); }

    /** Standard kick-off formation, {@code perTeam} robots each. */
    public void setupTeams(int perTeam) {
        World w = world;
        w.lock().lock();
        try {
            for (Robot r : List.copyOf(w.robots())) w.removeRobot(r.team, r.id);
            double hl = w.field().halfLength(), hw = w.field().halfWidth();
            for (int i = 0; i < perTeam; i++) {
                double y = perTeam == 1 ? 0 : -hw * 0.8 + i * (hw * 1.6 / (perTeam - 1));
                double x = (i == 0) ? hl - 0.3 : hl * (0.25 + 0.6 * (i % 3) / 3.0);
                w.addRobot(Robot.Team.YELLOW, i, specsFor(Robot.Team.YELLOW, i), x, y, Math.PI);
                w.addRobot(Robot.Team.BLUE, i, specsFor(Robot.Team.BLUE, i), -x, y, 0);
            }
            w.ball().teleport(new Vector3D(0, 0, w.ball().radius()), Vector3D.ZERO, true);
        } finally {
            w.lock().unlock();
        }
    }

    public void start() {
        if (running) return;
        running = true;
        thread = new Thread(this::loop, "simulation");
        thread.setDaemon(true);
        thread.start();
    }

    public void stop() {
        running = false;
        if (thread != null) thread.interrupt();
    }

    private void loop() {
        long last = System.nanoTime();
        while (running) {
            long now = System.nanoTime();
            double real = Math.min((now - last) * 1e-9, 0.05);
            last = now;
            if (!paused) {
                for (Camera.DetectionFrame f : advance(real * speed)) frameListeners.forEach(l -> l.accept(f));
            }
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                return;
            }
        }
    }

    /** Advance simulation by dt seconds, returning the camera frames captured meanwhile. */
    public List<Camera.DetectionFrame> advance(double dt) {
        World w = world;
        w.lock().lock();
        try {
            double chunk = 0.004;
            double left = dt;
            List<Camera.DetectionFrame> out = new java.util.ArrayList<>();
            while (left > 1e-9) {
                double d = Math.min(chunk, left);
                w.advance(d);
                out.addAll(vision.tick(w));
                if (telemetry != null) telemetry.sample(w);
                left -= d;
            }
            return out;
        } finally {
            w.lock().unlock();
        }
    }
}
