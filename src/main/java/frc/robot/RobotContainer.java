// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drivetrain.ToggleShiftingCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.ShifterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem driveTrainSubsystem  = new DriveTrainSubsystem();
  public Joystick driverStationJoystick;
  public ShifterSubsystem shifterSubsystem = new ShifterSubsystem();

  public RobotContainer() {
    // Configure the button bindings
    driverStationJoystick = new Joystick(0);

    switch (driveTrainSubsystem.getDriveMode()) {
      case TANK:
        driveTrainSubsystem.setDefaultCommand(
            new RunCommand(() -> driveTrainSubsystem.tankDrive(getLeftY(), getRightY()), driveTrainSubsystem));
        break;
      case CHEEZY:
        driveTrainSubsystem.setDefaultCommand(
            new RunCommand(() -> driveTrainSubsystem.cheezyDrive(getLeftY(), getRightX()), driveTrainSubsystem));
        break;
    }


    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    setJoystickButtonWhenPressed(driverStationJoystick, 12, new ToggleShiftingCommand(shifterSubsystem, driveTrainSubsystem));
  }


  private double getLeftY(){
    return -driverStationJoystick.getRawAxis(0);
  }

  private double getRightY(){
    return -driverStationJoystick.getRawAxis(2);
  }

  private double getRightX(){
    return driverStationJoystick.getRawAxis(3);
  }

  private void setJoystickButtonWhenPressed(Joystick joystick, int button, CommandBase command) {
    new JoystickButton(joystick, button).whenPressed(command);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          AutoConstants.ksVolts, 
          AutoConstants.kvVoltSecondsPerMeters,
          AutoConstants.kaVoltSecondsSquaredPerMeter),
          AutoConstants.kinematics,
          10);

    TrajectoryConfig config =
      new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(AutoConstants.kinematics)
        .addConstraint(autoVoltageConstraint);

    // s curve 
    Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      new Pose2d(3, 0, new Rotation2d(0)),
      config);

    RamseteCommand ramseteCommand = 
      new RamseteCommand(
        exampleTrajectory,
       driveTrainSubsystem::getPose,
       new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
       new SimpleMotorFeedforward(
         AutoConstants.ksVolts,
         AutoConstants.kvVoltSecondsPerMeters,
         AutoConstants.kaVoltSecondsSquaredPerMeter),
      AutoConstants.kinematics,
      driveTrainSubsystem::getWheelSpeeds,
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      driveTrainSubsystem::tankDriveVolts,
      driveTrainSubsystem);

  driveTrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

  return ramseteCommand.andThen(() -> driveTrainSubsystem.tankDriveVolts(0, 0));
}
  
}
