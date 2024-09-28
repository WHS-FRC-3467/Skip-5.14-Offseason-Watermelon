// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterJoint;

import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ShooterJoint extends SubsystemBase {

    /** Intake subsystem singleton. For superstructure. */
    private static ShooterJoint instance = null;

    private final ShooterJointIO io;
    private final ShooterJointIOInputsAutoLogged inputs = new ShooterJointIOInputsAutoLogged();
    private final ArmFeedforward ffModel;

    @RequiredArgsConstructor
    @Getter
    public enum State {
        STOW(() -> 0.0),
        PODIUM(() -> 30.0),
        SUBWOOFER(() -> 20.0),
        CLIMBCLEARANCE(() -> 20.0),
        DYNAMIC(()-> RobotState.getInstance().getShotAngle());

        private final DoubleSupplier outputSupplier;

        private double getStateOutput() {
            return outputSupplier.getAsDouble();
        }
    }

    @Getter
    @Setter
    private State state = State.STOW;

    private boolean debug = false;

    private final DutyCycleOut m_percent = new DutyCycleOut(0);
    private final NeutralOut m_neutral = new NeutralOut();

    private final double upperLimitDegrees = 80;
    private final double lowerLimitDegrees = 0;
    private final double maxVelocity = 1;
    private final double maxAcceleration = 1;
    private ProfiledPIDController pidController = new ProfiledPIDController(0, 0, 0,
        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    private double goalAngle;
    private double currentAngle;
    private ArmFeedforward ff = new ArmFeedforward(0, 0, 0, 0);
    private double output = 0;

    /**
    * Creates a new ShooterJoint.
    *
    * @return a subsystem instance if there isn't one already.
    */
    public static ShooterJoint getInstance() {
        if (instance == null) {
            
            switch (Constants.currentMode) {
                case REAL:
                    instance = new ShooterJoint(new ShooterJointIOTalonFX());
                    break;
                case SIM:
                    instance = new ShooterJoint(new ShooterJointIOSim());
                    break;
                default:
                    instance = new ShooterJoint(new ShooterJointIOTalonFX());
                    break;
          };

        }

        return instance;
    }

    /** Creates a new ComplexSubsystem. */
    public ShooterJoint(ShooterJointIO io) {

        this.io = io;
        switch (Constants.currentMode) {
            //For PID: Added kS, kV, and kA arguments
            case REAL:
                ffModel = new ArmFeedforward(0.0, 0.0, 0, 0); // Need to find
                io.configurePID(2.1, 0.0, 0.0, 0.1, 0.125, 0.01);
                break;
            case REPLAY:
                ffModel = new ArmFeedforward(0.0, 0.0, 0, 0); // Need to find
                io.configurePID(2.1, 0.0, 0.0, 0.1, 0.125, 0.01);
                break;
            case SIM:
                ffModel = new ArmFeedforward(0.0, 0.0, 0, 0); // Need to find
                io.configurePID(0.5, 0.0, 0.0, 0.0, 0.0, 0.0);
                break;
            default:
                ffModel = new ArmFeedforward(0.0, 0.0, 0, 0); // Need to find
                break;
    }

  }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        io.updateInputs(inputs);  // This will update the hardware logged
        Logger.processInputs("ShooterJoint", inputs);  //Updates the inputs in the logger

        goalAngle = MathUtil.clamp(state.getStateOutput(), lowerLimitDegrees, upperLimitDegrees);

        if (state == State.STOW && pidController.atGoal()) {
          io.stop();
        } else {
            output = pidController.calculate(currentAngle, goalAngle) + ff.calculate(inputs.position, inputs.velocity);
            io.setVoltage(output);
        }

    }


    /* Run Open loop at the specified voltage */
    public void runVolts(double volts) {
        io.setVoltage(volts);
    }
  
    public Command setStateCommand(State state) {
        Logger.recordOutput("Shooter Joint State", state.toString());
        return startEnd(() -> this.state = state, () -> this.state = State.STOW);
    }

}
