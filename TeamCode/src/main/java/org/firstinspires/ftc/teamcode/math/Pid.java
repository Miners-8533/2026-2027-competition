package org.firstinspires.ftc.teamcode.math;

/** A PID controller. The gains are passed on every call so that Dashboard edits act at once. */
public final class Pid {
    private double integral;
    private double lastError;
    private boolean first = true;

    /**
     * @param iLimit the most output the integral term may add, either way
     */
    public double update(double error, double dt, double kP, double kI, double kD, double iLimit) {
        if (kI != 0 && dt > 0) {
            integral += error * dt;
            double lim = Math.abs(iLimit / kI);
            integral = Angle.clamp(integral, -lim, lim);
        } else if (kI == 0) {
            integral = 0;
        }
        double d = (first || dt <= 0) ? 0 : (error - lastError) / dt;
        first = false;
        lastError = error;
        return kP * error + kI * integral + kD * d;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
        first = true;
    }
}
