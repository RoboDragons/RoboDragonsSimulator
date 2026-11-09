package ssl.sim.robot;

import ssl.sim.ball.Ball;

/** A part of a robot (drive train, kicker, dribbler...) that exerts forces during each physics step. */
public interface RobotComponent {
    void update(Robot robot, Ball ball, double dt);
}
