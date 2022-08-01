// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

//import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class DriveTrain extends SubsystemBase {
  private WPI_VictorSPX m_left1, m_left2, m_right1, m_right2;
  private DifferentialDrive drive;

  //private final AHRS gyro = new AHRS(SPI.Port.kMXP); 
  //private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  
  private static final Timer timer = new Timer();
  
  private double previous_time;

  private double totalLeftWheelDistanceMeters;
  private double totalRightWheelDistanceMeters;

  //private DifferentialDriveOdometry odometry;

  /** Creates a new DriveTrain and initializes motor controllers. */
  public DriveTrain() {

    timer.start();

    m_left1 = new WPI_VictorSPX(Constants.motor_left1);
    m_left2 = new WPI_VictorSPX(Constants.motor_left2);
    m_right1 = new WPI_VictorSPX(Constants.motor_right1);
    m_right2 = new WPI_VictorSPX(Constants.motor_right2);

    m_left1.enableVoltageCompensation(true);
    m_left2.enableVoltageCompensation(true);
    m_right1.enableVoltageCompensation(true);
    m_right2.enableVoltageCompensation(true);

    m_right1.setInverted(true); // one side runs clockwise, other runs counterclockwise
    m_right2.setInverted(true); // since the motors are facing opposite directions

    m_left1.setInverted(false); // one side runs clockwise, other runs counterclockwise
    m_left2.setInverted(false); // since the motors are facing opposite directions

    m_left1.setNeutralMode(NeutralMode.Brake); // only one motor brakes on each side for a weaker brake effect
    m_left2.setNeutralMode(NeutralMode.Coast);
    m_right1.setNeutralMode(NeutralMode.Brake);
    m_right2.setNeutralMode(NeutralMode.Coast);

    m_left2.follow(m_left1); // syncs the motors on each side
    m_right2.follow(m_right1);

    m_right1.setSelectedSensorPosition(0); // for encoder
    m_left1.setSelectedSensorPosition(0);

    m_left1.configOpenloopRamp(0.2); // limits acceleration, takes 0.4 seconds to accelerate from 0 to 100%
    m_left2.configOpenloopRamp(0.2); // (helps keep robot from rocking around violently every time driver stops)
    m_right1.configOpenloopRamp(0.2);
    m_right2.configOpenloopRamp(0.2);

    drive = new DifferentialDrive(m_left1,m_right1); // only need to input one motor per side, the other two follow them

    setWheelPositionZero();
    //odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  }

  /**
   * arcadeDrive based teleoperated control. Left stick throttle, right stick turn.
   * @param throttle Input for speed.
   * @param steer Input for turning speed.
   * @param maxSpeed Max speed [0.0..1.0].
   * @param maxTurnSpeed Max turn speed [0.0..1.0].
   * @param squareInputs If set, squares inputs for precision at low speeds.
   */
  public void manualDrive(double throttle, double steer, double maxSpeed, double maxTurnSpeed, boolean squareInputs) {
    if(squareInputs) {
      throttle = Math.copySign(throttle*throttle, throttle);
      steer = Math.copySign(steer*steer, steer);
    } // does not use arcadeDrive's squareInputs parameter to preserve max speeds
    drive.arcadeDrive(maxSpeed*throttle, maxTurnSpeed*steer, false);
  }

  //Driving using voltages
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_left1.setVoltage(leftVolts);
    m_right1.setVoltage(rightVolts);
    drive.feed();
  }
  
  //unused, tbh dunno why we added this
  public void stop() {
    drive.stopMotor();
  }
  
  //pretty much unused
  private void setWheelPositionZero() {
    totalLeftWheelDistanceMeters = 0;
    totalRightWheelDistanceMeters = 0;
  } 

  /* 
  public void resetOdometry(Pose2d pose) {
    setWheelPositionZero();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }
  */

  //we do not have encoders on the everybot, so instead, we are just going to use our gyro only
  /*
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
      gyro.getVelocityX(), 
      gyro.getVelocityY(), 
      gyro.getVelocityZ()
    );

    return Constants.kinematics.toWheelSpeeds(chassisSpeeds);
  }
  */
  /*
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  */

  private void trackLeftAndRightDistance(DifferentialDriveWheelSpeeds wheelSpeeds) {
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;

    double rightVelocity = wheelSpeeds.rightMetersPerSecond;

    double current_time = Timer.getFPGATimestamp();

    double timeElapsedBetweenLoops = current_time - previous_time;

    double leftWheelDistanceMeters = leftVelocity*timeElapsedBetweenLoops;
    double rightWheelDistanceMeters = rightVelocity*timeElapsedBetweenLoops;

    totalLeftWheelDistanceMeters += leftWheelDistanceMeters;
    totalRightWheelDistanceMeters += rightWheelDistanceMeters;
  }
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Output",m_left1.get());
    SmartDashboard.putNumber("Right Output",m_right1.get());
    SmartDashboard.putNumber("Left Position",totalLeftWheelDistanceMeters);
    SmartDashboard.putNumber("Right Position",totalRightWheelDistanceMeters);

    //DifferentialDriveWheelSpeeds wheelSpeeds = getWheelSpeeds();
    //trackLeftAndRightDistance(wheelSpeeds);

    //odometry.update(
    //  gyro.getRotation2d(), 
    //  totalLeftWheelDistanceMeters, 
    //  totalRightWheelDistanceMeters
    //);

  }

  //Unused
  /**
   * Drives and turns at a set speed.
   * @param speed [-1.0..1.0].
   * @param turnSpeed [-1.0..1.0].
   */
  public void autonomousDrive(double speed, double turnSpeed) {
    drive.arcadeDrive(speed,turnSpeed,false);
  }
  /**
   * Sets the speeds for each side of tank drive individually (for easier usage with encoders).
   * @param leftSpeed [-1.0..1.0]
   * @param rightSpeed [-1.0..1.0]
   */
  public void autonomousTank(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed,rightSpeed);
  }

}
