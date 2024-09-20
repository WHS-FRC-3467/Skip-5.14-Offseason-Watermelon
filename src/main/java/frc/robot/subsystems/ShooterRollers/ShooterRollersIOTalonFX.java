package frc.robot.subsystems.ShooterRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ShooterRollersIOTalonFX implements ShooterRollersIO {
    private static final double GEAR_RATIO = 0.0;   // Need to get this information

    private final TalonFX leader = new TalonFX(Constants.ExampleSubsystemConstants.ID_Motor);
    private final StatusSignal<Double> leaderPosition = leader.getPosition();
    private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
    private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
    private final StatusSignal<Double> leaderCurrent = leader.getSupplyCurrent();

    public ShooterRollersIOTalonFX() {
        var config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 100.0;
        config.CurrentLimits.SupplyCurrentThreshold = 100;
        config.CurrentLimits.SupplyTimeThreshold = 0.1; 
        config.CurrentLimits.SupplyCurrentLimitEnable = true; 
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.StatorCurrentLimitEnable = true;         
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0; 

        config.MotionMagic.MotionMagicAcceleration = 40.0; // Start small acceleration and jerk and go faster
        config.MotionMagic.MotionMagicJerk = 400.0;
        leader.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
        leader.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterRollersIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
        inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / GEAR_RATIO;
        inputs.velocityRadPerSec =
            Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / GEAR_RATIO;
        inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
        inputs.currentAmps =
            new double[] {leaderCurrent.getValueAsDouble()};
    }

    @Override
    public void setVoltage(double volts) {
        leader.setControl(new VoltageOut(volts));
    }

    @Override
    public void setVelocity(double velocityRadPerSec, double ffVolts) {
        leader.setControl(
            new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec),
                0.0,
                true,
                ffVolts,
                0,
                false,
                false,
                false));
    }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    leader.getConfigurator().apply(config);
  }
}   