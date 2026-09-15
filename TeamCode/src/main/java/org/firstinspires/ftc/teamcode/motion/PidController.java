package org.firstinspires.ftc.teamcode.motion;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Plain scalar PID with anti-windup and an optional static (stiction) term.
 *
 * Deliberately general - it knows nothing about angles or poses. Callers that need
 * angle wrap-around must compute the error themselves (see {@link DriveToPose}).
 *
 * Time step is measured internally. Call {@link #reset()} before the first
 * {@link #calculate} of a new move.
 */
public final class PidController {

    public double kP;
    public double kI;
    public double kD;

    /** Added in the direction of the output whenever the output is non-trivial. Beats stiction. */
    public double kStatic = 0.0;
    /** Anti-windup: the integral term is clamped to +/- this. */
    public double maxIntegral = 0.25;
    /** Output is clamped to +/- this. */
    public double maxOutput = 1.0;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0.0;
    private double integral = 0.0;
    private boolean first = true;

    public PidController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void reset() {
        first = true;
        integral = 0.0;
        lastError = 0.0;
        timer.reset();
    }

    public double calculate(double error) {
        double dt = timer.seconds();
        timer.reset();

        double derivative = 0.0;
        if (!first && dt > 1e-6) {
            derivative = (error - lastError) / dt;
            integral += error * dt;
            integral = clamp(integral, -maxIntegral, maxIntegral);
        }
        first = false;
        lastError = error;

        double output = kP * error + kI * integral + kD * derivative;
        if (Math.abs(output) > 1e-6) {
            output += Math.copySign(kStatic, output);
        }
        return clamp(output, -maxOutput, maxOutput);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
