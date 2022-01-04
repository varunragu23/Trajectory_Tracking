// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.sensors.PigeonIMU;

import java.lang.Math.*;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.

  WPI_TalonFX leftMotor1 = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
  WPI_TalonFX leftMotor2 = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);
  WPI_TalonFX rightMotor1 = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
  WPI_TalonFX rightMotor2 = new WPI_TalonFX(DriveConstants.kRightMotor2Port);

  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(
          leftMotor1,
          leftMotor2);


  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(
          rightMotor1,
          rightMotor2);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  // private final Encoder m_leftEncoder =
  //     new Encoder(
  //         DriveConstants.kLeftEncoderPorts[0],
  //         DriveConstants.kLeftEncoderPorts[1],
  //         DriveConstants.kLeftEncoderReversed);

  // // The right-side drive encoder
  // private final Encoder m_rightEncoder =
  //     new Encoder(
  //         DriveConstants.kRightEncoderPorts[0],
  //         DriveConstants.kRightEncoderPorts[1],
  //         DriveConstants.kRightEncoderReversed);
  // The gyro sensor
  // https://www.ctr-electronics.com/downloads/pdf/Pigeon%20IMU%20User's%20Guide.pdf
  private final PigeonIMU p_gyro = new PigeonIMU(0);


  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    leftMotor1.configFactoryDefault();
    leftMotor2.configFactoryDefault();
    rightMotor1.configFactoryDefault();
    rightMotor2.configFactoryDefault();

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    leftMotor1.configAllSettings(configs);
    leftMotor2.configAllSettings(configs);
    rightMotor1.configAllSettings(configs);
    rightMotor2.configAllSettings(configs);

    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    leftMotor1.setNeutralMode(NeutralMode.Brake);
    leftMotor2.setNeutralMode(NeutralMode.Brake);
    rightMotor1.setNeutralMode(NeutralMode.Brake);
    rightMotor2.setNeutralMode(NeutralMode.Brake);

    leftMotor1.setInverted(DriveConstants.kLeftEncoderReversed);
    leftMotor2.setInverted(DriveConstants.kLeftEncoderReversed);

    rightMotor1.setInverted(DriveConstants.kRightEncoderReversed);
    rightMotor2.setInverted(DriveConstants.kRightEncoderReversed);

    m_drive.setRightSideInverted(false);



    // Sets the distance per pulse for the encoders
    // --------------do we need to do this????-------------
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    // double[] ypr_deg = new double[3];
    // p_gyro.getYawPitchRoll(ypr_deg);
    // Rotation2d rot = new Rotation2d(ypr_deg[0]);
    m_odometry = new DifferentialDriveOdometry(get2dRotation());
    // DifferentialDriveOdometry is initialized with a 2drotation
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        get2dRotation(), getLeftEncoderDistance(), getRightEncoderDistance());
  }

  private Rotation2d get2dRotation() {
    // .getCompassHeading()??
    double[] ypr_deg = new double[3];
    p_gyro.getYawPitchRoll(ypr_deg);

    // double radians = p_gyro.getCompassHeading() * (double)Math.PI / (double)180.000;

    // new Rotation2d();
    // Rotation2d rot = new Rotation2d.fromDegrees(ypr_deg[0]);

    return Rotation2d.fromDegrees(ypr_deg[0]);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, get2dRotation());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    // m_leftEncoder.reset();
    // m_rightEncoder.reset();

    leftMotor1.setSelectedSensorPosition(0);
    rightMotor1.setSelectedSensorPosition(0);
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  public double getLeftEncoderDistance(){
    return leftMotor1.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse;
    
  }

  public double getRightEncoderDistance(){
    return rightMotor1.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse;

  }

  public double getRightEncoderRate() {
    return rightMotor1.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse / 60;

  }

  public double getLeftEncoderRate() {
    return leftMotor1.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse / 60;
    
  }


  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  // public double getAverageEncoderDistance() {
    // return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  // }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  // public Encoder getLeftEncoder() {
    // return m_leftEncoder;
  // }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  // public Encoder getRightEncoder() {
    // return m_rightEncoder;
  // }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    p_gyro.addYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return get2dRotation().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    double[] xyz_dps = new double[3];
    p_gyro.getRawGyro(xyz_dps);
    return -xyz_dps[2];
  }
}
