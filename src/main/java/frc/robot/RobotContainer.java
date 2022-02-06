// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.drivetrain.ToggleShiftingCommand;
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
  public Joystick driverStationJoystick;

  public ShifterSubsystem shifterSubsystem;
  private final DriveTrainSubsystem drivetrainSubsystem;

  public RobotContainer() {
    driverStationJoystick = new Joystick(IOConstants.DRIVER_STATION_JOY);
    shifterSubsystem = new ShifterSubsystem();
    drivetrainSubsystem = new DriveTrainSubsystem();

    switch (drivetrainSubsystem.getDriveMode()) {
      case TANK:
        drivetrainSubsystem.setDefaultCommand(
            new RunCommand(() -> drivetrainSubsystem.tankDrive(getLeftY(), getRightY()), drivetrainSubsystem));
        break;
      case CHEEZY:
        drivetrainSubsystem.setDefaultCommand(
            new RunCommand(() -> drivetrainSubsystem.cheezyDrive(getLeftY(), getRightX()), drivetrainSubsystem));
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
    setJoystickButtonWhenPressed(driverStationJoystick, 12, new ToggleShiftingCommand(shifterSubsystem, drivetrainSubsystem));
  }

  private double getLeftY() {
    return -driverStationJoystick.getRawAxis(0);
  }

  private double getRightY() {
    return -driverStationJoystick.getRawAxis(2);
  }

  private double getRightX() {
    return driverStationJoystick.getRawAxis(3);
  }

  private void setJoystickButtonWhenPressed(Joystick joystick, int button, CommandBase command) {
    new JoystickButton(joystick, button).whenPressed(command);
  }
  /*
   * public Command getAutonomousCommand() {
   * // An ExampleCommand will run in autonomous
   * return m_autoCommand;
   * }
   */
}
