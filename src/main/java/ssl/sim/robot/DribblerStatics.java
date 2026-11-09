package ssl.sim.robot;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.ArrayRealVector;
import ssl.sim.ball.Ball;

/**
 * Static force balance of a ball held (at rest relative to the robot) by a backspinning roller.
 * Computed in the robot's x-z plane for the hold position defined by {@link RobotSpecs}.
 *
 * <h3>Three point hold: roller + chipper plate + ground</h3>
 * The roller slips (its surface is far faster than the ball's), the chipper plate and the ground slip too, so all three
 * friction forces are Coulomb limits (mu * N). Unknowns: roller normal Nr, chipper normal Nc, ground normal Ng;
 * equations: sum Fx = 0, sum Fz = 0 and the steady-spin torque balance (roller friction torque = ground + chipper
 * friction torque).
 *
 * <h3>Two point hold: roller + ground</h3>
 * Without the chipper the torque balance cannot be met with a slipping roller, so the ball spins up until its surface
 * speed equals the roller's (roller sticks): the roller friction Fr is a free unknown (direction along the roller
 * surface motion) that must stay below mu_r * Nr. Unknowns: Nr, Fr, Ng.
 *
 * <p>Contact directions (ball receives the forces): roller normal (cos a, -sin a) i.e. forward-down; chipper normal
 * (cos d, sin d), i.e. forward-up; ground normal up. Friction acts against the backspin slip: the ground pulls the ball
 * towards the robot, the roller pulls its upper-rear surface backward-down.
 */
public final class DribblerStatics {
    private DribblerStatics() {}

    public record Result(boolean stable, double rollerNormal, double chipperNormal, double groundNormal,
                         double rollerFriction, String note) {}

    public record Both(Result threePoint, Result twoPoint) {}

    public static Both analyse(RobotSpecs s, double muRoller, double muChipper, double muGround, double ballMass) {
        return new Both(threePoint(s, muRoller, muChipper, muGround, ballMass), twoPoint(s, muRoller, muGround, ballMass));
    }

    public static Both analyse(Robot r, Ball.Params ball) {
        return analyse(r.specs(), r.dribbler().rubber.mu(), r.chipper().material().mu(), ball.muSlide, ball.mass);
    }

    public static Result threePoint(RobotSpecs s, double muR, double muC, double muG, double mass) {
        double a = Math.toRadians(s.rollerContactAngleDeg), d = Math.toRadians(s.chipperContactAngleDeg);
        double w = mass * Ball.G;
        double[][] m = {
                {Math.cos(a) - muR * Math.sin(a), Math.cos(d) - muC * Math.sin(d), -muG},
                {-Math.sin(a) - muR * Math.cos(a), Math.sin(d) + muC * Math.cos(d), 1},
                {-muR, muC, muG}};
        double[] x = solve(m, new double[]{0, w, 0});
        if (x == null) return new Result(false, 0, 0, 0, 0, "singular system");
        boolean ok = x[0] >= 0 && x[1] >= 0 && x[2] >= 0;
        return new Result(ok, x[0], x[1], x[2], muR * x[0], ok ? "stable" : "no non-negative solution (ball not held)");
    }

    public static Result twoPoint(RobotSpecs s, double muR, double muG, double mass) {
        double a = Math.toRadians(s.rollerContactAngleDeg);
        double w = mass * Ball.G;
        // unknowns: Nr, Fr (friction along (-sin a, -cos a)), Ng
        double[][] m = {
                {Math.cos(a), -Math.sin(a), -muG},
                {-Math.sin(a), -Math.cos(a), 1},
                {0, -1, muG}};
        double[] x = solve(m, new double[]{0, w, 0});
        if (x == null) return new Result(false, 0, 0, 0, 0, "singular system");
        boolean ok = x[0] >= 0 && x[2] >= 0 && x[1] >= 0 && x[1] <= muR * x[0];
        String note = ok ? "stable" : x[1] > muR * x[0] ? "roller friction demand exceeds mu*N (ball slips off)" : "no non-negative solution";
        return new Result(ok, x[0], 0, x[2], x[1], note);
    }

    private static double[] solve(double[][] m, double[] rhs) {
        try {
            DecompositionSolver solver = new LUDecomposition(new Array2DRowRealMatrix(m)).getSolver();
            RealVector v = solver.solve(new ArrayRealVector(rhs));
            return v.toArray();
        } catch (RuntimeException e) {
            return null;
        }
    }
}
