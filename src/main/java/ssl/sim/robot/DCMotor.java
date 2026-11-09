package ssl.sim.robot;

/** Brushed/brushless DC motor: torque = kt * (V - ke*w)/R, limited by a current limit. */
public class DCMotor {
    public final double kt;        // torque constant [Nm/A]
    public final double ke;        // back-EMF constant [V s/rad]
    public final double resistance; // [ohm]
    public final double maxVoltage;
    public final double maxCurrent;
    public final double rotorInertia; // [kg m^2]
    public final double viscousFriction; // [Nm s/rad]

    public DCMotor(double kt, double resistance, double maxVoltage, double maxCurrent, double rotorInertia, double viscousFriction) {
        this.kt = kt;
        this.ke = kt;
        this.resistance = resistance;
        this.maxVoltage = maxVoltage;
        this.maxCurrent = maxCurrent;
        this.rotorInertia = rotorInertia;
        this.viscousFriction = viscousFriction;
    }

    /** Shaft torque [Nm] for an applied voltage and shaft speed [rad/s]. */
    public double torque(double voltage, double speed) {
        double v = Math.max(-maxVoltage, Math.min(maxVoltage, voltage));
        double i = (v - ke * speed) / resistance;
        i = Math.max(-maxCurrent, Math.min(maxCurrent, i));
        return kt * i - viscousFriction * speed;
    }

    /** Maxon EC45-flat 50 W like motor used for omni wheels. */
    public static DCMotor wheelMotor() {
        return new DCMotor(0.0276, 0.6, 24.0, 10.0, 1.0e-5, 1.0e-5);
    }

    /** Small high speed motor for the dribbler roller (12 V, ~12000 rpm no-load). */
    public static DCMotor dribblerMotor() {
        return new DCMotor(0.0095, 0.5, 12.0, 15.0, 1.0e-5, 1.0e-6);
    }
}
