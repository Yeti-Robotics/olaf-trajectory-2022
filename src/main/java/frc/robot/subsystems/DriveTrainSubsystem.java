// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */

  private WPI_TalonFX leftFalcon1, leftFalcon2, rightFalcon1, rightFalcon2;

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;
  private DriveMode driveMode;
  private AHRS gyro;

  public enum DriveMode {
    TANK, CHEEZY;
  }

  public DriveTrainSubsystem() {
    leftFalcon1 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_1);
    leftFalcon2 = new WPI_TalonFX(DriveConstants.LEFT_FALCON_2);
    rightFalcon1 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_1);
    rightFalcon2 = new WPI_TalonFX(DriveConstants.RIGHT_FALCON_2);

    rightFalcon1.setInverted(true);

    leftFalcon2.follow(leftFalcon1);
    leftFalcon2.setInverted(InvertType.FollowMaster);
    rightFalcon2.follow(rightFalcon1);
    rightFalcon2.setInverted(InvertType.FollowMaster);

    // leftMotors = new MotorControllerGroup(leftFalcon1, leftFalcon2);
    // rightMotors = new MotorControllerGroup(rightFalcon1, rightFalcon2);

    // rightMotors.setInverted(true);

    drive = new DifferentialDrive(leftFalcon1, rightFalcon1);
    drive.setDeadband(0.05);

    leftFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    rightFalcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    leftFalcon1.setNeutralMode(NeutralMode.Brake);
    leftFalcon2.setNeutralMode(NeutralMode.Brake);
    rightFalcon1.setNeutralMode(NeutralMode.Brake);
    rightFalcon2.setNeutralMode(NeutralMode.Brake);
    resetEncoders();

    gyro = new AHRS(Port.kOnboard);
    zeroHeading();
    
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    driveMode = DriveMode.CHEEZY;
  }

  @Override
  public void periodic() {
    odometry.update(
        gyro.getRotation2d(), getLeftEncoder(), getRightEncoder());

    // System.out.printf("GYRO: %f \n LEFT ENCODER: %f \n RIGHT ENCODER: %f \n LEFT ENCODER VELOCITY: %f \n", gyro.getRotation2d().getDegrees(), getLeftEncoder(), getRightEncoder(), getLeftEncoderRate());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    // System.out.println("LEFT VOLTS: " + leftVolts);
    // System.out.println("RIGHT VOLTS: " + rightVolts);
    leftFalcon1.setVoltage(leftVolts);
    rightFalcon1.setVoltage(rightVolts);
    drive.feed();
  }

  public void tankDrive(double leftpower, double rightpower) {
    drive.tankDrive(leftpower, rightpower);
  }

  public void cheezyDrive(double straight, double turn) {
    drive.curvatureDrive(straight, -turn, false);
  }

  public void stopDrive() {
    leftFalcon1.set(ControlMode.PercentOutput, 0);
    rightFalcon1.set(ControlMode.PercentOutput, 0);
  }

  public double getLeftEncoder() {
    return (leftFalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE)
        / (ShifterSubsystem.getShifterPosition() == ShifterSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO
            : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getRightEncoder() {
    return (rightFalcon1.getSelectedSensorPosition() * (DriveConstants.DISTANCE_PER_PULSE)
        / (ShifterSubsystem.getShifterPosition() == ShifterSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO
            : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getLeftEncoderRate() {
    return (leftFalcon1.getSelectedSensorVelocity() * (DriveConstants.DISTANCE_PER_PULSE) * 10
    / (ShifterSubsystem.getShifterPosition() == ShifterSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO
        : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getRightEncoderRate() {
    return (rightFalcon1.getSelectedSensorVelocity() * (DriveConstants.DISTANCE_PER_PULSE) * 10
    / (ShifterSubsystem.getShifterPosition() == ShifterSubsystem.ShiftStatus.HIGH ? DriveConstants.HIGH_GEAR_RATIO
        : DriveConstants.LOW_GEAR_RATIO));
  }

  public double getAverageEncoder() {
    return ((getLeftEncoder() + getRightEncoder()) / 2);
  }

  public void resetEncoders() {
    leftFalcon1.setSelectedSensorPosition(0);
    rightFalcon1.setSelectedSensorPosition(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getRawEncoder() {
    return leftFalcon1.getSelectedSensorPosition();
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }

  public DriveMode getDriveMode() {
    return driveMode;
  }

  public void setDriveMode(DriveMode driveMode) {
    this.driveMode = driveMode;
  }

}
