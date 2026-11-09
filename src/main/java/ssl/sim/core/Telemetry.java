package ssl.sim.core;

import ssl.sim.robot.Robot;
import ssl.sim.robot.RobotCommand;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Optional CSV logger for analysing an external controller: every received robot command (with wall-clock arrival
 * time) and a 50 Hz state sample of all robots that have received a command.
 */
public class Telemetry implements AutoCloseable {
    private final BufferedWriter commands, states;
    private final long t0 = System.nanoTime();
    private double nextSample;
    private final java.util.Set<String> commanded = new java.util.HashSet<>();
    private final java.util.Map<String, double[]> last = new java.util.HashMap<>();

    public Telemetry(Path dir) throws IOException {
        Files.createDirectories(dir);
        commands = Files.newBufferedWriter(dir.resolve("commands.csv"));
        states = Files.newBufferedWriter(dir.resolve("states.csv"));
        commands.write("wall,sim,team,id,mode,a,b,c,kick,kick_angle,dribbler\n");
        states.write("wall,sim,team,id,x,y,theta,vf,vl,w,cmd_f,cmd_l,cmd_a,sp_f,sp_l,sp_a\n");
    }

    private double wall() { return (System.nanoTime() - t0) * 1e-9; }

    public synchronized void command(double simTime, Robot.Team team, int id, RobotCommand c) {
        String mode;
        double a = 0, b = 0, d = 0;
        switch (c.move()) {
            case RobotCommand.LocalVelocity l -> { mode = "local"; a = l.forward(); b = l.left(); d = l.angular(); }
            case RobotCommand.GlobalVelocity g -> { mode = "global"; a = g.x(); b = g.y(); d = g.angular(); }
            case RobotCommand.WheelVelocity w -> { mode = "wheel"; a = w.frontRight(); b = w.backRight(); d = w.backLeft(); }
            case null -> mode = "none";
        }
        commanded.add(team + ":" + id);
        last.put(team + ":" + id, new double[]{a, b, d});
        try {
            commands.write(String.format("%.5f,%.5f,%s,%d,%s,%.4f,%.4f,%.4f,%.3f,%.1f,%.0f%n", wall(), simTime, team, id, mode, a, b, d,
                    c.kickSpeed(), c.kickAngleDeg(), c.dribblerRpm()));
        } catch (IOException ignored) {
        }
    }

    /** Called after each physics chunk with the world locked. */
    public synchronized void sample(World w) {
        if (w.time() < nextSample) return;
        nextSample = w.time() + 0.02;
        for (Robot r : w.robots()) {
            String k = r.team + ":" + r.id;
            if (!commanded.contains(k)) continue;
            double cs = Math.cos(r.heading()), sn = Math.sin(r.heading());
            double vx = r.velocity().getX(), vy = r.velocity().getY();
            double[] c = last.get(k), sp = r.drive().setpointLocal();
            try {
                states.write(String.format("%.5f,%.5f,%s,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f%n", wall(), w.time(),
                        r.team, r.id, r.x(), r.y(), r.heading(), cs * vx + sn * vy, -sn * vx + cs * vy, r.angularVelocity(),
                        c[0], c[1], c[2], sp[0], sp[1], sp[2]));
            } catch (IOException ignored) {
            }
        }
    }

    @Override
    public synchronized void close() {
        try {
            commands.close();
            states.close();
        } catch (IOException ignored) {
        }
    }
}
