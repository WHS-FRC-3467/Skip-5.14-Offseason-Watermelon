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
import frc.robot.RobotState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import frc.robot.Constants.ClimberConstants;

// Make sure to get all the imports needed and change Climber in constants if needed
public class Climber extends SubsystemBase{
    @RequiredArgsConstructor
    @Getter
  public enum State {
    //
    HOME(() -> 0.0),
    //Following two are rough esitamtes
    CLIMB(() -> 120.0),
    DOWN(() -> 20.0),

    OUT(() -> 90.0);

    private final DoubleSupplier outputSupplier;

    private double getStateOutput() {
      return outputSupplier.getAsDouble();
    }
  }

  @Getter
  @Setter
  private State state = State.HOME;

 
    
    TalonFX m_leftShooterMotor = new TalonFX(ClimberConstants.ID_ClimberLeader);
    TalonFX m_rightShooterMotor = new TalonFX(ClimberConstants.ID_ClimberFollower);
    public Climber() {

    }
  

  
    public Command setStateCommand(State state) {
      return startEnd(() -> this.state = state, () -> this.state = State.HOME);
    }
  
      private void displayInfo(boolean debug) {
      if (debug) {
        //smart dashboard stuff
      }
  
    }
}
