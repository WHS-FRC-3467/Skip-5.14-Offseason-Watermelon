package frc.robot.subsystems.ShooterJoint;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ShooterJointIOTalonFX implements ShooterJointIO {
    /* This section will define the motor characterization as well as the settings we want to apply */
    private final TalonFX leader = new TalonFX(Constants.ShooterJointConstants.ID_SHOOTER_JOINT); //
    //private final TalonFX follower = new TalonFX(1); // Not using a follower

    private final StatusSignal<Double> leaderPosition = leader.getPosition(); //
    private final StatusSignal<Double> leaderVelocity = leader.getVelocity(); // 
    private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage(); //
    private final StatusSignal<Double> leaderCurrent = leader.getSupplyCurrent(); //
    //private final StatusSignal<Double> followerCurrent = follower.getSupplyCurrent(); //

    public ShooterJointIOTalonFX() {
        
        var config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 100.0;
        config.CurrentLimits.SupplyCurrentThreshold = 100.0;
        config.CurrentLimits.SupplyTimeThreshold = 0.1; 
        config.CurrentLimits.SupplyCurrentLimitEnable = true; 
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.StatorCurrentLimitEnable = true; 
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; //Coast || Brake
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0; 

            // set Motion Magic settings
        /* Actual values have yet to be tested */
        config.MotionMagic.MotionMagicCruiseVelocity = 8; // Target cruise velocity of 80 rps
        config.MotionMagic.MotionMagicAcceleration = 16; // Target acceleration of 16 rps/s
        config.MotionMagic.MotionMagicJerk = 160; // Target jerk of 160 rps/s/s

        leader.getConfigurator().apply(config);
        //follower.getConfigurator().apply(config);
        //follower.setControl(new Follower(leader.getDeviceID(), false));

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent/*, followerCurrent*/);
        leader.optimizeBusUtilization();
        //follower.optimizeBusUtilization();
    }

    // Now we will override those methods form the IO Layer again
    @Override
    public void updateInputs(ShooterJointIOInputs inputs) {
        // you always want to make sure you are updating the global variables that carry out through the IO layers
        // Here we are making sure that we also update the Hardware as well
        BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent/*, followerCurrent */);

        inputs.test = 0.2;
        inputs.position = leaderPosition.getValueAsDouble();
        inputs.velocity = leaderVelocity.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        leader.setControl(new VoltageOut(volts));
    }

    @Override
    public void stop() {
        leader.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD, double kS, double kV, double kA) {
        var config = new Slot0Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kS = kS;
        config.kV = kV;
        config.kA = kA;
        leader.getConfigurator().apply(config);
    }
}