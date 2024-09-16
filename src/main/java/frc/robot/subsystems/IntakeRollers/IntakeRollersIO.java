package frc.robot.subsystems.IntakeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
    @AutoLog
    public static class IntakeRollersIOInputs {
        // This is where we will establish variables
        public double test = 0.0;
    }

    /* Updates the set of loggable inputs */
    public default void updateInputs(IntakeRollersIOInputs inputs) {}

    /* Run Open Loop at the specified voltage. */
    public default void setVoltage(double volts) {}

    /* Stop in open loop */
    public default void stop() {}

    /* Set velocity PID constants */
    public default void configurePID(double kP, double kI, double kD) {}
}
