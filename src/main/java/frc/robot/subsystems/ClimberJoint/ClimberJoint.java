// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberJoint;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ClimberJoint extends SubsystemBase {

  private final ClimberJointIO io;
  private final ClimberJointIOInputsAutoLogged inputs = new ClimberJointIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  @RequiredArgsConstructor
  @Getter
  public enum State {
    ON(1.0),
    OFF(0.0);

    private final double output;
  }

  @Getter
  @Setter
  private State state = State.OFF;

  private boolean debug = false;

  TalonFX m_motor = new TalonFX(Constants.ExampleSubsystemConstants.ID_Motor);
  private final DutyCycleOut m_percent = new DutyCycleOut(0);
  private final NeutralOut m_brake = new NeutralOut();

  /** Creates a new SimpleSubsystem. */
  public ClimberJoint(ClimberJointIO io) {
    this.io = io;
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0); // Need to find
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0); // Need to find
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0); // Need to find
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0); // Need to find
        break;
    }

    m_motor.getConfigurator().apply(Constants.ExampleSubsystemConstants.motorConfig());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);  // This will update the hardware logged

    if (state == State.OFF) {
      m_motor.setControl(m_brake);
    } else {
      m_motor.setControl(m_percent.withOutput(state.getOutput()));
    }

    displayInfo(debug);
  }

  /* Run Open loop at the specified voltage */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.OFF);
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString("SimpleSubsystem State ", state.toString());
      SmartDashboard.putNumber("SimpleSubsystem Setpoint ", state.getOutput());
      SmartDashboard.putNumber("SimpleSubsystem Output ", m_motor.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("SimpleSubsystem Current Draw", m_motor.getSupplyCurrent().getValueAsDouble());
    }

  }
}