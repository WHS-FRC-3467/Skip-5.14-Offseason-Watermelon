// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollerConstants;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import java.util.function.DoubleSupplier;

public class IntakeRollers extends SubsystemBase {

    /** Intake subsystem singleton. For superstructure. */
    private static IntakeRollers instance = null;

    @RequiredArgsConstructor
    @Getter
    public enum State{

        FWD (() -> IntakeRollerConstants.k_INTAKE_FWD_SPEED),
        REV (() -> IntakeRollerConstants.k_INTAKE_REV_SPEED),
        OFF(() -> 0.0);

        private final DoubleSupplier outputSupplier;

        private double getStateOutput() {
            return outputSupplier.getAsDouble();
        }
    }

    @Getter
    @Setter
    private State state = State.OFF;

    // Initalize Motors and Beam Break
    TalonFX m_roller = new TalonFX(IntakeRollerConstants.ID_IntakeRoller);

    // For superstructure
    /**
    * Returns the intake subsystem instance.
    *
    * @return the intake subsystem instance.
    */
    public static IntakeRollers getInstance() {
        if (instance == null) {
            instance = new IntakeRollers();
        }

        return instance;
    }

    /** Creates a new IntakeSubsystem. */
    public IntakeRollers() {
        
        m_roller.getConfigurator().apply(IntakeRollerConstants.motorConfig());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        displayInfo(true);

        m_roller.set(state.getStateOutput());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * Example command factory method. Periodic tells the intake to run according to the state
     *
     * @return a command setting the intake state to the argument
     */
    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.OFF);
    }

    public void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString("Intake Roller State", getState().toString());
            SmartDashboard.putNumber("Intake Roller Setpoint", state.getStateOutput());
            SmartDashboard.putNumber("Intake Roller speed", m_roller.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Intake Roller Current Draw", m_roller.getSupplyCurrent().getValueAsDouble());
        }
    }
}