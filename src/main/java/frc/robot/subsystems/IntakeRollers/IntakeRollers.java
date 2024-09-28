// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeRollers;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeRollerConstants;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import java.util.function.DoubleSupplier;

public class IntakeRollers extends SubsystemBase {

    /** Intake subsystem singleton. For superstructure. */
    private static IntakeRollers instance = null;

    private final IntakeRollersIO io;
    private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

    @RequiredArgsConstructor
    @Getter
    public enum State{

        FWD (() -> -0.6*12),
        REV (() -> 0.3*12),
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
            switch (Constants.currentMode) {
                case REAL:
                    instance = new IntakeRollers(new IntakeRollersIOTalonFX());
                    break;
                case SIM:
                    instance = new IntakeRollers(new IntakeRollersIOSim());
                    break;
                default:
                    instance = new IntakeRollers(new IntakeRollersIOTalonFX());
                    break;
            };
        }

        return instance;
    }

    /** Creates a new IntakeSubsystem. */
    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;

        // We need to establish what mode the software is running on to determine if PID needs to change
        // There are different PID values and FeedForward values since the physics simulator does not fully match
        // the real robot
        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                io.configurePID(1.0, 0.0, 0.0);
                break;
            case SIM:
                io.configurePID(0.5, 0.0, 0.0);
                break;
            default:
                break;
        }
        // MJW 9/13/2024: We no longer establish the hardware settings in the constants folder
        // instead the user will define the motor settings inside of the IO layer TalonFx
        // m_roller.getConfigurator().apply(IntakeRollerConstants.motorConfig());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);  // Goes to the IO layer and updates the variables from Hardware/Sim
        Logger.processInputs("IntakeRollers", inputs);  //Updates the inputs in the logger
        
        // This method will be called once per scheduler run
        displayInfo(true);

        io.setVoltage(state.getStateOutput());
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