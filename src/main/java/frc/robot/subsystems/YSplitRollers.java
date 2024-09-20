// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class YSplitRollers extends SubsystemBase {

  @RequiredArgsConstructor
  @Getter
  public enum State {
    OFF(0.0,0.0), //No movement
    SHOOTER(1.0,1.0), //Roller 1 intakes, Roller 2 sends to shooter
    REVSHOOTER(-1.0,-1.0), //Reverse out of shooter
    ELEVATOR(1.0,-1.0), //Roller 1 intake, Roller 2 sends to elevator
    REVELEVATOR(-1.0,1.0); //Reverse out of elevator

    private final double roller1; //From the intake
    private final double roller2; //Decides to shooter or to elevator
  }

  @Getter
  @Setter
  private State state = State.OFF;

  private boolean debug = true;

  TalonFX m_roller1 = new TalonFX(Constants.YSplitRollersConstants.ID_YSPLIT_ROLLER1);
  TalonFX m_roller2 = new TalonFX(Constants.YSplitRollersConstants.ID_YSPLIT_ROLLER2);
  private final DutyCycleOut m_percent = new DutyCycleOut(0);
  private final NeutralOut m_brake = new NeutralOut();

  /** Creates a new YSplitRollers. */
  public YSplitRollers() {
    m_roller1.getConfigurator().apply(Constants.YSplitRollersConstants.motorConfig());
    m_roller2.getConfigurator().apply(Constants.YSplitRollersConstants.motorConfig());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (state == State.OFF) {
      m_roller1.setControl(m_brake);
      m_roller2.setControl(m_brake);
    } else {
      m_roller1.setControl(m_percent.withOutput(state.getRoller1()));
      m_roller2.setControl(m_percent.withOutput(state.getRoller2()));
    }

    displayInfo(debug);
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.OFF);
  }

  private void displayInfo(boolean debug) {
    if (debug) {
      SmartDashboard.putString("YSplitRollers State ", state.toString());
      SmartDashboard.putNumber("YSplitRollers Roller1 Setpoint ", state.getRoller1());
      SmartDashboard.putNumber("YSplitRollers Roller2 Setpoint ", state.getRoller2());
      SmartDashboard.putNumber("YSplitRollers Roller1 Output ", m_roller1.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("YSplitRollers Roller2 Output ", m_roller2.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("YSplitRollers Roller1 Current Draw", m_roller1.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber("YSplitRollers Roller2 Current Draw", m_roller2.getSupplyCurrent().getValueAsDouble());
    }

  }
}