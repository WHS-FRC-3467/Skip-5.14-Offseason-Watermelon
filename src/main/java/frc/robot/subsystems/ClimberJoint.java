package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.RobotState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import frc.robot.Constants.ClimberConstants;


// Make sure to get all the imports needed and change Climber in constants if needed
public class ClimberJoint extends SubsystemBase{
    @RequiredArgsConstructor
    @Getter
  public enum State {
    
    //Following are rough esitamtes
    CLIMB(() -> 120.0),
    DOWN(() -> 10.0),
    STOW(() -> 0.0);

    private final DoubleSupplier outputSupplier;

    private double getStateOutput() {
      return outputSupplier.getAsDouble();
    }
  }

  @Getter
  @Setter
  private State state = State.STOW;


   
    private ProfiledPIDController pidController = new ProfiledPIDController(1, 0, 0,
        new TrapezoidProfile.Constraints(ClimberConstants.maxVelocity, ClimberConstants.maxAcceleration));
    private double currentAngle;
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0);
    private double climberStowed = 0.0;
    private double climberMaxExtension = 0.48;
    private double climberSetpoint;
    private double climberPull = 0.02;
    private double rotationsPerUnitDistance = 8.0 / (Units.inchesToMeters(0.655) * Math.PI);
    private final NeutralOut m_neutral = new NeutralOut();
    private double output = 0;
    private double goalAngle;

    TalonFX m_climberLeaderMotor = new TalonFX(ClimberConstants.ID_ClimberLeader);
    TalonFX m_climberFollowerMotor = new TalonFX(ClimberConstants.ID_ClimberFollower);
    DutyCycleEncoder m_climberEncoder = new DutyCycleEncoder(ClimberConstants.k_CLIMBER_ENCODER_ID);

    private final NeutralOut m_brake = new NeutralOut();    

    public ClimberJoint() {

        m_climberLeaderMotor.getConfigurator().apply(ClimberConstants.motorConfig());
        m_climberFollowerMotor.getConfigurator().apply(ClimberConstants.motorConfig());
        m_climberFollowerMotor.setControl(new Follower(m_climberLeaderMotor.getDeviceID(), true));
            /* Set range of duty cycle encoder in fractions of rotation */
        m_climberEncoder.setDutyCycleRange(ClimberConstants.kDuty_Cycle_Min, ClimberConstants.kDuty_Cycle_Max);

            // Position offset in duty cycle, not rads
        m_climberEncoder.setPositionOffset(ClimberConstants.k_CLIMBER_HORIZONTAL_OFFSET_DUTYCYCLE);
        // this sets the distance per rotation to be equal to 2pi radians
        m_climberEncoder.setDistancePerRotation(Math.PI*2);

      pidController.setTolerance(ClimberConstants.tolerance);


    }
  
    public void periodic() {
      displayInfo(true);

      if (state == State.STOW) {
        m_climberLeaderMotor.setControl(m_brake);
      } else {
        output = pidController.calculate(currentAngle, goalAngle) + ff.calculate(0, 0);
      }
        
      climberSetpoint = MathUtil.clamp(state.getStateOutput(), climberStowed, climberMaxExtension);
      if (state == State.STOW && pidController.atGoal()) {

      }
    }
  
    public Command setStateCommand(State state) {
      return startEnd(() -> this.state = state, () -> this.state = State.STOW);
    }
  
    private void displayInfo(boolean debug) {
      if (debug) {
        //smart dashboard stuff
        SmartDashboard.putNumber("Climber angle", currentAngle);
        SmartDashboard.putNumber("Climber Setpoint ", state.getStateOutput());
        
      }

    }
}
