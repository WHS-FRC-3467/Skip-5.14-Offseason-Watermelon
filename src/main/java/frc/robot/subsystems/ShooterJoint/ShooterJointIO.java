package frc.robot.subsystems.ShooterJoint;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterJointIO {
    
    @AutoLog
    public static class ShooterJointIOInputs {
        // This is where we will establish variables
        public double test = 0.0;
    }

    /* Updates the set of loggable inputs */
    public default void updateInputs(ShooterJointIOInputs inputs) {}

    /* Run Open Loop at the specified voltage. */
    public default void setVoltage(double volts) {}

    /* Stop in open loop */
    public default void stop() {}

    /* Set velocity PID constants */
    public default void configurePID(double kP, double kI, double kD, double kS, double kV, double kA) {}
}