package frc.robot.subsystems.ClimberJoint;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberJointIO {
    @AutoLog
    public static class ClimberJointIOInputs {
        //Insert Variables to monitor
        public double motorVoltage = 0.0;
        public double currentDraw = 0.0;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
    }
    
    /* Updates the set of loggable inputs */
    public default void updateInputs(ClimberJointIOInputs inputs) {}

    /* Run Open Loop at the specified voltage */
    public default void setVoltage(double volts) {}

    /** Run closed loop at the specified velocity. */
    public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

    /* Stop in open loop */
    public default void stop() {}

    /* Set velocity PID constants */
    public default void configurePID(double kP, double kI, double kD) {}
}
