// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FlywheelConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ShooterRollers extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {
        OFF     (()-> 0.0),
        PASSTHROUGH    (()-> 10.0), // Poop & Scoot
        AMP     (()-> 40.0), 
        SUBWOOFER(()-> 27.0),
        SHOOT   (()-> 75.0), // Default
        FEED    (()-> 28.0),
        REVERSE (()-> -20), // Hopefully never have to use this irl
        AIMING  (()-> 75.0); // Robotstate to be created

        private final DoubleSupplier velocitySupplier;

        private double getStateOutput() {
            return velocitySupplier.getAsDouble();
        }
    }

    @Getter
    @Setter
    private State state = State.OFF;

      // Initialize motor controllers
    TalonFX m_flywheel = new TalonFX(FlywheelConstants.ID_Flywheel); 

    private final double speedMax = 100.0;
    private final double speedMin = 0.0;
    private final double maxVelocity = 1;
    private final double maxAcceleration = 1;
    
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private double goalSpeed;
    private double currentAngle;
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0);
    private double output = 0;

    NeutralOut m_neutralOut = new NeutralOut();


    /** Creates a new Flywheel. */
    public ShooterRollers() {
        m_flywheel.getConfigurator().apply(FlywheelConstants.shooterMotorConfig(m_flywheel.getDeviceID()));
    }

    @Override
    public void periodic() {
      
        displayInfo(true);

        if (state == State.OFF) {
            m_flywheel.setControl(m_neutralOut);
        } else {
            goalSpeed = MathUtil.clamp(state.getStateOutput(), speedMin, speedMax);  
            // create a velocity closed-loop request, voltage output, slot 0 configs
            m_flywheel.setControl(m_request.withVelocity(goalSpeed).withFeedForward(0.5));
        }

    }

    public Command setStateCommand(State state) {
        return runOnce(() -> this.state = state);
    }

    public void displayInfo(boolean debug) {
        if (debug) {
            //SmartDashboard.putBoolean("Shooter at speed", atGoal());
            SmartDashboard.putString("Flywheel state", getState().toString());
            SmartDashboard.putNumber("Flywheel Setpoint", state.getStateOutput());
            SmartDashboard.putNumber("Flywheel Current draw", m_flywheel.getSupplyCurrent().getValueAsDouble());
        }
    }
}
