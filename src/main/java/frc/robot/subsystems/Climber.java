package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
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
public class Climber extends SubsystemBase{
    @RequiredArgsConstructor
    @Getter
  public enum State {
    
    //Following are rough esitamtes
    CLIMB(() -> 120.0),
    DOWN(() -> 20.0),
    STOW(() -> 0.0);

    private final DoubleSupplier outputSupplier;

    private double getStateOutput() {
      return outputSupplier.getAsDouble();
    }
  }

  @Getter
  @Setter
  private State state = State.STOW;


  private final double maxVelocity = 1;
  private final double maxAcceleration = 1;
  private ProfiledPIDController pidController = new ProfiledPIDController(1, 0, 0,
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
  private double currentAngle;
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0);
  private double climberStowed = 0.0;
  private double climberSetpoint = 0.4;
  private double climberMaxExtension = 0.48;
  private double climberPull = 0.02;
  private double rotationsPerUnitDistance = 8.0 / (Units.inchesToMeters(0.655) * Math.PI);
  

    
    TalonFX m_climberLeaderMotor = new TalonFX(ClimberConstants.ID_ClimberLeader);
    TalonFX m_climberFollowerMotor = new TalonFX(ClimberConstants.ID_ClimberFollower);
    public Climber() {

    }
  
    public void periodic() {
      displayInfo(true);
        
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

      }
  
    }
}
