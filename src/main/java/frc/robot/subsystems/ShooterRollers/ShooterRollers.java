// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterRollers;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import frc.robot.Constants.ShooterRollersConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ShooterRollers extends SubsystemBase {

    private final ShooterRollersIO io;
    private final ShooterRollersIOInputsAutoLogged inputs = new ShooterRollersIOInputsAutoLogged();
    private final SimpleMotorFeedforward ffModel;

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

    private final double speedMax = 100.0;
    private final double speedMin = 0.0;
    private final double maxVelocity = 1;
    private final double maxAcceleration = 1;
    
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    // create a Motion Magic request, voltage output
    //final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

    private double goalSpeed;
    private double currentAngle;
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0);
    private double output = 0;

    NeutralOut m_neutralOut = new NeutralOut();


    /** Creates a new Flywheel. */
    public ShooterRollers(ShooterRollersIO io) {
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
        // m_flywheel.getConfigurator().apply(ShooterRollersConstants.shooterMotorConfig(m_flywheel.getDeviceID()));
        }
    }

    @Override
    public void periodic() {
      
        if (state == State.OFF) {
            io.stop();
        } else {
            goalSpeed = MathUtil.clamp(state.getStateOutput(), speedMin, speedMax);  
            // create a Motion Magic velocity closed-loop request, voltage output, slot 0 configs OR just use regular Velocity Voltage:
            io.setVelocity(goalSpeed, 0);
        }

    }

    public Command setStateCommand(State state) {
        return runOnce(() -> this.state = state);
    }
}