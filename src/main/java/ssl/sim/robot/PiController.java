package ssl.sim.robot;

/** PI controller with output saturation and conditional anti-windup. */
public class PiController {
    private final double kp, ki, limit;
    private double integral;

    public PiController(double kp, double ki, double limit) {
        this.kp = kp;
        this.ki = ki;
        this.limit = limit;
    }

    public double update(double error, double dt) {
        double u = kp * error + ki * integral;
        double sat = Math.max(-limit, Math.min(limit, u));
        if (u == sat || (u > 0) != (error > 0)) integral += error * dt;
        return sat;
    }

    public void reset() { integral = 0; }
}
