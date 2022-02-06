// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShifterSubsystem;
import frc.robot.subsystems.ShifterSubsystem.ShiftStatus;

public class ToggleShiftingCommand extends CommandBase {
  private final ShifterSubsystem shifterSubsystem;
  private DriveTrainSubsystem driveTrainSubsystem;

  /** Creates a new ToggleShiftingCommand. */
  public ToggleShiftingCommand(ShifterSubsystem shifterSubsystem, DriveTrainSubsystem driveTrainSubsystem) {
    this.shifterSubsystem = shifterSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
    addRequirements(shifterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrainSubsystem.resetEncoders();
    if (ShifterSubsystem.getShifterPosition() == ShiftStatus.HIGH) {
      shifterSubsystem.shiftDown();
    } else {
      shifterSubsystem.shiftUp();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
