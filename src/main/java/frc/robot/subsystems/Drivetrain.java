/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.music.Orchestra;

import java.util.ArrayList;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
//import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase {
  // left side motors
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(DriveConstants.kLeftMaster);
  private final WPI_TalonFX leftSlave = new WPI_TalonFX(DriveConstants.kLeftSlave);

  // right side motors
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(DriveConstants.kRightMaster);
  private final WPI_TalonFX rightSlave = new WPI_TalonFX(DriveConstants.kRightSlave);

  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMaster, leftSlave);
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightMaster, rightMaster);

  // the drivebase
  private final DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

  //the gyro
  private final Gyro m_gyro = new ADIS16448_IMU();

  //music!
  private Orchestra orchestra;
  private ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  //kinematics and path following stuff
  public static final double kTrackwidthMeters = 0.69;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

  // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
  // These characterization values MUST be determined either experimentally or theoretically
  // for *your* robot's drive.
  // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
  // values for your robot.
  public static final double ksVolts = 0.22;
  public static final double kvVoltSecondsPerMeter = 1.98;
  public static final double kaVoltSecondsSquaredPerMeter = 0.2;

  // Example value only - as above, this must be tuned for your drive!
  public static final double kPDriveVel = 8.5;

  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  private boolean swapDrive;

  public Drivetrain() {
    //reset talons to factory defaults just in case of a swap or a setting changed in phoenix tuner
    leftMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlave.configFactoryDefault();

    // all of this is to prevent battery brownouts
    leftMaster.configVoltageCompSaturation(DriveConstants.kVoltageCompLevel);
    leftMaster.enableVoltageCompensation(true);
    leftMaster.configOpenloopRamp(DriveConstants.kRampCoefficient);
    leftSlave.configVoltageCompSaturation(DriveConstants.kVoltageCompLevel);
    leftSlave.enableVoltageCompensation(true);
    leftSlave.configOpenloopRamp(DriveConstants.kRampCoefficient);
    rightMaster.configVoltageCompSaturation(DriveConstants.kVoltageCompLevel);
    rightMaster.enableVoltageCompensation(true);
    rightMaster.configOpenloopRamp(DriveConstants.kRampCoefficient);    
    rightSlave.configVoltageCompSaturation(DriveConstants.kVoltageCompLevel);
    rightSlave.enableVoltageCompensation(true);
    rightSlave.configOpenloopRamp(DriveConstants.kRampCoefficient);
		
		/* Configure output and sensor direction */
    leftMaster.setInverted(true);
    leftSlave.setInverted(true);
    rightMaster.setInverted(false);
    rightSlave.setInverted(false);

    //make sure all of the motors brake when no input is applied
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);

    instruments.add(leftMaster);
    instruments.add(leftSlave);
    instruments.add(rightMaster);
    instruments.add(rightSlave);

    orchestra = new Orchestra(instruments);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    orchestra.loadMusic("cleanup.chrp");

    m_drive.setSafetyEnabled(false); //motor safety was getting mad because we are overrunning loop times, so i tunred it off -J

    m_drive.setRightSideInverted(false); //we are doing motor inverts above manually, so changing it in wpilib is redundant

    swapDrive = false;
    SmartDashboard.putBoolean("drive swapped?", swapDrive); //for telling the driver if he has engaged drive swapped

  }

  public void arcadeDrive(double fwd, double rot) {
    //drive swap logic
    if(swapDrive){
      m_drive.arcadeDrive(-fwd, (-1)*(rot-0));
    }
    else{
      m_drive.arcadeDrive(fwd, (-1)*(rot+0), true); // Squaring values
    }
  }

  //drive each side individually (for auto)
  public void drive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  public void stop(){
    leftMaster.set(ControlMode.PercentOutput, 0);
    rightMaster.set(ControlMode.PercentOutput, 0);
  }

  public void log(){
    SmartDashboard.putNumber("encoder distance in inches", getAverageEncoderDistanceInInches());
  }

  public void playMusic(){
    orchestra.play();
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoder() {
    return leftMaster.getSensorCollection().getIntegratedSensorPosition();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoder() {
    return rightMaster.getSensorCollection().getIntegratedSensorPosition();
  }

  /** Zero integrated encoders on Talon */
  public void resetEncoders() {
    leftMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.DriveConstants.kTimeoutMs);
    rightMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.DriveConstants.kTimeoutMs);
    System.out.println("[Integrated Sensors] All sensors are zeroed.\n");
  }

  public void calibrateGyro(){
    m_gyro.calibrate();
  }

  public void resetAllSensors() {
    leftMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.DriveConstants.kTimeoutMs);
    rightMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.DriveConstants.kTimeoutMs);
    m_gyro.reset();
  }

  public double getAverageEncoderDistance() {
    return ((getLeftEncoder() + getRightEncoder()) / 2.0);
  }

  public double getAverageEncoderDistanceInInches() {
    return ((getLeftEncoder() + getRightEncoder()) / 2.0)/2048/10.75; //need to check math here, but this seems right
  }



  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftMaster.getSensorCollection().getIntegratedSensorPosition(),
                      rightMaster.getSensorCollection().getIntegratedSensorPosition());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void toggleSwap(){
    if(swapDrive){
      swapDrive = false;
      SmartDashboard.putBoolean("drive swapped?", swapDrive);
    }
    else{
      swapDrive = true;
      SmartDashboard.putBoolean("drive swapped?", swapDrive);
    }
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftMaster.getSensorCollection().getIntegratedSensorVelocity(), rightMaster.getSensorCollection().getIntegratedSensorVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
