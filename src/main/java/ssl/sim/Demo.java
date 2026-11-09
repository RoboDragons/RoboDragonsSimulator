package ssl.sim;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import ssl.sim.core.Simulator;
import ssl.sim.robot.Robot;
import ssl.sim.robot.RobotCommand;

/** Small scripted scene for looking at the dribbler and kicker: blue 0 drives onto the ball, holds it, then kicks. */
final class Demo {
    private Demo() {}

    static void start(Simulator sim) {
        var w = sim.world();
        w.lock().lock();
        try {
            Robot r = w.findRobot(Robot.Team.BLUE, 0);
            if (r == null) return;
            r.setPose(-1.5, 0, 0);
            w.ball().teleport(new Vector3D(-1.0, 0, w.ball().radius()), Vector3D.ZERO, true);
            r.applyCommand(new RobotCommand(new RobotCommand.LocalVelocity(0.5, 0, 0), 0, 0, 9000));
        } finally {
            w.lock().unlock();
        }
        Thread t = new Thread(() -> {
            try {
                Thread.sleep(2000);
                send(sim, new RobotCommand(new RobotCommand.LocalVelocity(0, 0, 0), 0, 0, 9000));
                Thread.sleep(2500);
                send(sim, new RobotCommand(new RobotCommand.LocalVelocity(0, 0, 0), 4.0, 0, 9000));
                Thread.sleep(300);
                send(sim, new RobotCommand(new RobotCommand.LocalVelocity(0, 0, 0), 0, 0, 0));
            } catch (InterruptedException ignored) {
            }
        }, "demo");
        t.setDaemon(true);
        t.start();
    }

    private static void send(Simulator sim, RobotCommand c) {
        var w = sim.world();
        w.lock().lock();
        try {
            Robot r = w.findRobot(Robot.Team.BLUE, 0);
            if (r != null) r.applyCommand(c);
        } finally {
            w.lock().unlock();
        }
    }
}
