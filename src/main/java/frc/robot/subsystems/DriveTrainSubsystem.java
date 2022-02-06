// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */

  private WPI_TalonFX leftFalcon1, leftFalcon2, rightFalcon1, rightFalcon2;
  private PigeonIMU gyro;

  public final DifferentialDrive drive;
  private DriveMode driveMode;


  public enum DriveMode{
    TANK, CHEEZY;
  }

  public DriveTrainSubsystem() {
    leftFalcon1 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_1);
    leftFalcon2 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_2);
    rightFalcon1 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_1);
    rightFalcon2 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_2);

    rightFalcon1.setInverted(true);
    rightFalcon2.follow(rightFalcon1);
    rightFalcon2.setInverted(InvertType.FollowMaster);
    leftFalcon2.follow(leftFalcon1);
    leftFalcon2.setInverted(InvertType.FollowMaster);

    drive = new DifferentialDrive(leftFalcon1, rightFalcon1);
    drive.setDeadband(0.05);
  
    leftFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    rightFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    leftFalcon1.setNeutralMode(NeutralMode.Brake);
    rightFalcon1.setNeutralMode(NeutralMode.Brake);
    resetEncoders();

    gyro = new PigeonIMU(13);

    driveMode = DriveMode.TANK;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftpower, double rightpower){

    drive.tankDrive(leftpower, rightpower);

  }

  public void cheezyDrive(double straight, double turn){
    drive.curvatureDrive(straight, -turn, false);
  }

  public void stopDrive(){

    leftFalcon1.set(ControlMode.PercentOutput, 0);
    rightFalcon1.set(ControlMode.PercentOutput, 0);
  }

  public void resetEncoders(){
    leftFalcon1.setSelectedSensorPosition(0);
    rightFalcon1.setSelectedSensorPosition(0);
  
  }

  // public double getLeftEncoder() {
  //   return (leftFalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE)  / (ShiftingGearSubsystem.getShifterPosition() == ShiftingGearSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO : DriveConstants.LOW_GEAR_RATIO));
  // }

  // public double getRightEncoder() {
  //   return (- rightFalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE) / (ShiftingGearSubsystem.getShifterPosition() == ShiftingGearSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO : DriveConstants.LOW_GEAR_RATIO));
  // }

  // public double getAverageEncoder(){
  //   return ((getLeftEncoder()+getRightEncoder())/2);
  // } use when ready


  public void setMaxOutput(double maxOutput){
    drive.setMaxOutput(maxOutput);
  }

  public double getAngle(){
    double [] ypr_deg = new double[3];
    gyro.getYawPitchRoll(ypr_deg);
    return ypr_deg[0];
  }

  public void resetGyro(){
    gyro.setYaw(0);
  }
  
  public double getRawEncoder() {
    return leftFalcon1.getSelectedSensorPosition(); 
  }


  public DriveMode getDriveMode(){
    return driveMode;
  }

  public void setDriveMode(DriveMode driveMode){
    this.driveMode = driveMode;
  }
}
