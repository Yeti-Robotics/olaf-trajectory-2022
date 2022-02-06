// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class ShifterSubsystem extends SubsystemBase {
  private DoubleSolenoid shifter;
    public enum ShiftStatus{
        HIGH, LOW
    }
    public static ShiftStatus shiftStatus;
    
  public ShifterSubsystem() {
    shifter = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM, DriveConstants.SHIFTER_SOLENOID[0], DriveConstants.SHIFTER_SOLENOID[1]
      );
    shiftStatus = ShiftStatus.LOW;
    
  }

  public void shiftUp() {
    shifter.set(DoubleSolenoid.Value.kForward);
    shiftStatus = ShiftStatus.HIGH;
  }
  public void shiftDown() {
    shifter.set(DoubleSolenoid.Value.kReverse);
    shiftStatus = ShiftStatus.LOW;
  }

  public static ShiftStatus getShifterPosition() {
    return shiftStatus;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
