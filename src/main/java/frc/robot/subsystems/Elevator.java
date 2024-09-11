// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.ElevatorConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Elevator extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State { //0 angle is straight horizontal, -20 is on the hardstops
    STOW(() -> 0.0),
    SCORE(() -> 90.0),
    DYNAMIC(() -> RobotState.getInstance().getShotAngle());

    private final DoubleSupplier outputSupplier;

    private double getStateOutput() {
      return Units.degreesToRadians(outputSupplier.getAsDouble());
    }
  }

  @Getter
  @Setter
  private State state = State.STOW;


  //Create a on RIO Profile PID Controller, adding constraints to limit max vel and accel 
  private ProfiledPIDController pidController = new ProfiledPIDController(18, 0, 0.2,
      new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration));

  private ElevatorFeedforward ff = new ElevatorFeedforward(0.5, 0.4, 2.5, 0.01);

  private double goalAngle;
  private double output = 0;

  TalonFX m_ElevatorMotor = new TalonFX(ElevatorConstants.ID_ElevatorLeader);
  TalonFX m_ElevatorFollowerMotor = new TalonFX(ElevatorConstants.ID_ElevatorFollower);

  private VoltageOut m_VoltageOutput = new VoltageOut(0.0);
  private NeutralOut m_neutralOut = new NeutralOut();

  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(ElevatorConstants.ID_ElevatorAbsEncoder);

  private Debouncer m_debounce = new Debouncer(.1);
  


  /** Creates a new ElevatorSubsystem. */
  public Elevator() {

    m_ElevatorMotor.getConfigurator().apply(ElevatorConstants.motorConfig());
    m_ElevatorFollowerMotor.getConfigurator().apply(ElevatorConstants.motorConfig());
    m_ElevatorFollowerMotor.setControl(new Follower(ElevatorConstants.ID_ElevatorLeader, true));

    m_encoder.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0); 
    m_encoder.setDistancePerRotation(2*Math.PI); //Define 1 full rotation to be 2 Pi
    m_encoder.setPositionOffset(0.50666666); //Encoder offset to make horizontal report 0 deg
    

    pidController.setTolerance(ElevatorConstants.tolerance);
    pidController.setIZone(100); // TODO: Figure out actual value

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    goalAngle = MathUtil.clamp(state.getStateOutput(), ElevatorConstants.lowerLimit, ElevatorConstants.upperLimit);
    pidController.setGoal(goalAngle);
    
    if (state == State.STOW && atGoal()) {
      m_ElevatorMotor.setControl(m_neutralOut);
    } else {
      output = pidController.calculate(m_encoder.getDistance()) + ff.calculate(goalAngle, 0);
      m_ElevatorMotor.setControl(m_VoltageOutput.withOutput(output));
    }

    displayInfo(true);

  }

  
  public boolean atGoal() {
    //return m_debounce.calculate(MathUtil.isNear(goalAngle, m_encoder.getDistance(), ElevatorConstants.tolerance));
    return m_debounce.calculate(pidController.atGoal());
    //return pidController.atGoal();
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> setState(state), () -> setState(State.STOW))
        .withName("Set Elevator State: " + state.toString());
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString("Elevator State ", state.toString());
      SmartDashboard.putNumber("Elevator Setpoint ", Units.radiansToDegrees(goalAngle));
      SmartDashboard.putNumber("Elevator Angle ", Units.radiansToDegrees(m_encoder.getDistance()));
      SmartDashboard.putBoolean("Elevator at Goal?", atGoal());
      SmartDashboard.putNumber("Elevator Current Draw", m_ElevatorMotor.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putData("Elevator PID",pidController);
    }

  }
}
