// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX m_leftPrimary = new WPI_TalonFX(DriveConstants.leftUpperMotorID);
  private final WPI_TalonFX m_rightPrimary = new WPI_TalonFX(DriveConstants.rightUpperMotorID);
  private final WPI_TalonFX m_leftSecondary = new WPI_TalonFX(DriveConstants.leftLowerMotorID);
  private final WPI_TalonFX m_rightSecondary = new WPI_TalonFX(DriveConstants.rightLowerMotorID);

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftPrimary, m_leftSecondary);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightPrimary, m_rightSecondary);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  private final DifferentialDriveOdometry m_odometry;

  public Drivetrain() {
    m_rightMotors.setInverted(true);
    m_leftMotors.setInverted(false);

    m_rightPrimary.setSelectedSensorPosition(0.0);
    m_leftPrimary.setSelectedSensorPosition(0.0);

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  public double pulsesToMeters(double counts) {
    return Units.feetToMeters(counts * (1.0 / 2048.0) * (1.0 / 18.0) * (0.5 / Math.PI));
  }

  public double velocityToMetersPerSecond(double rate) {
    return pulsesToMeters(rate * 10.0);
  }

  @Override
  public void periodic() {
    m_odometry.update(m_gyro.getRotation2d(), pulsesToMeters(m_leftPrimary.getSelectedSensorPosition()),
        pulsesToMeters(m_rightPrimary.getSelectedSensorPosition()));
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(velocityToMetersPerSecond(m_leftPrimary.getSelectedSensorVelocity()),
        velocityToMetersPerSecond(m_rightPrimary.getSelectedSensorVelocity()));
  }

  public void resetOdometry(Pose2d pose) {
    m_rightPrimary.setSelectedSensorPosition(0.0);
    m_leftPrimary.setSelectedSensorPosition(0.0);

    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_rightPrimary.setSelectedSensorPosition(0.0);
    m_leftPrimary.setSelectedSensorPosition(0.0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (pulsesToMeters(m_leftPrimary.getSelectedSensorPosition())
        + pulsesToMeters(m_rightPrimary.getSelectedSensorPosition())) / 2.0;
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
