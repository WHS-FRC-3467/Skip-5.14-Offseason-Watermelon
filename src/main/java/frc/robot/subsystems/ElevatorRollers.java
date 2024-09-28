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




public class ElevatorRollers extends SubsystemBase {


 @RequiredArgsConstructor
 @Getter
 public enum State {
   ON(1.0),
   REVERSE(-1.0),
   OFF(0.0);


   private final double ElevatorRollers;




 }


 @Getter
 @Setter
 private State state = State.OFF;
 private boolean debug = true;


 TalonFX m_elevatorRollers = new TalonFX(Constants.ElevatorRollersConstants.ID_ElevatorRollers);
 private final DutyCycleOut m_percent = new DutyCycleOut(0);
 private final NeutralOut m_brake = new NeutralOut();


 /** Creates a new SimpleSubsystem. */
 public ElevatorRollers() {
   m_elevatorRollers.getConfigurator().apply(Constants.ElevatorRollersConstants.motorConfig());


 }


 @Override
 public void periodic() {
   // This method will be called once per scheduler run
   displayInfo(debug);


   if (state == State.OFF) {
     m_elevatorRollers.setControl(m_brake);
   } else {
     m_elevatorRollers.setControl(m_percent.withOutput(state.getElevatorRollers()));
   }
 }


 public Command setStateCommand(State state) {
   return startEnd(() -> this.state = state, () -> this.state = State.OFF);
 }


 public void displayInfo(boolean debug){
     SmartDashboard.putString("ElevatorRollers State ", state.toString());
     SmartDashboard.putNumber("ElevatorRollers Setpoint ", state.getElevatorRollers());
     SmartDashboard.putNumber("ElevatorRollers Output ", m_elevatorRollers.getMotorVoltage().getValueAsDouble());
     SmartDashboard.putNumber("ElevatorRollers Current Draw", m_elevatorRollers.getSupplyCurrent().getValueAsDouble());
   }
 }




