// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LaserCanSensor;
import frc.robot.subsystems.SimpleSubsystem;
import frc.robot.subsystems.YSplitRollers;
import frc.robot.subsystems.ClimberJoint;

public class RobotContainer {
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick

  public final Drivetrain drivetrain = TunerConstants.DriveTrain;
  public final YSplitRollers ySplitRollers = new YSplitRollers();

  public final LaserCanSensor laserCanSensor1 = new LaserCanSensor(0);
  public final LaserCanSensor laserCanSensor2 = new LaserCanSensor(0);

  public final Trigger beamBreak1 = new Trigger(() -> laserCanSensor1.isClose());
  public final Trigger beamBreak2 = new Trigger(() -> laserCanSensor2.isClose());

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  // AUTO - Register Named Commands
  //NamedCommands.registerCommand("StopShooter", m_ShooterSubsystem.setStateCommand(ShooterState.STOP));
          // Build an auto chooser. This will use Commands.none() as the default option.
  autoChooser = AutoBuilder.buildAutoChooser();
          // Another option that allows you to specify the default auto by its name
          // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
  SmartDashboard.putData("Auto Chooser", autoChooser);

  private final Telemetry logger = new Telemetry(DriveConstants.MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.run(
        () -> drivetrain.setControllerInput(-joystick.getLeftY(), -joystick.getLeftX(), -joystick.getRightX())));
    
    // reset the field-centric heading on left bumper press
    //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    drivetrain.registerTelemetry(logger::telemeterize);


    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    /* joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse)); */

    //joystick.x().whileTrue(m_shooterpivot.setStateCommand(ShooterPivot.State.CLIMBERCLEARENCE).alongWith(Commands.waitUntil(m_shooterpivot::atGoal).andThen(m_climber.setStateCommand(ClimberJoint.State.CLIMB))));
    //joystick.b().whileTrue(m_climber.setStateCommand(ClimberJoint.State.DOWN));
  }

  public RobotContainer() {
    /* m_climber = new ClimberJoint();
    m_shooterpivot = new ShooterPivot(); */
    configureBindings();
  }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
     public Command getAutonomousCommand() {
      return autoChooser.getSelected();
  }

}
