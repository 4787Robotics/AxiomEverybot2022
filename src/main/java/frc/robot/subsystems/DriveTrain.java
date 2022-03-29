// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX m_left1;
  private WPI_TalonFX m_right1;
  private WPI_TalonFX m_left2;
  private WPI_TalonFX m_right2;

  
  private DifferentialDrive drive;
  
  /** Creates a new DriveTrain and initializes motor controllers. */
  public DriveTrain() {
    m_left1 = new WPI_TalonFX(Constants.motor_left1);
    m_left2 = new WPI_TalonFX(Constants.motor_left2);
    m_right1 = new WPI_TalonFX(Constants.motor_right1);
    m_right2 = new WPI_TalonFX(Constants.motor_right2);

    m_right1.setInverted(TalonFXInvertType.Clockwise);
    m_right2.setInverted(TalonFXInvertType.Clockwise);

    m_left1.setNeutralMode(NeutralMode.Coast);
    m_left2.setNeutralMode(NeutralMode.Coast);
    m_right1.setNeutralMode(NeutralMode.Coast);
    m_right2.setNeutralMode(NeutralMode.Coast);

    m_left2.follow(m_left1);
    m_right2.follow(m_right1);

    m_right1.setSelectedSensorPosition(0);
    m_left1.setSelectedSensorPosition(0);

    m_left1.configOpenloopRamp(0.7);
    m_left2.configOpenloopRamp(0.7);
    m_right1.configOpenloopRamp(0.7);
    m_right2.configOpenloopRamp(0.7);

    drive = new DifferentialDrive(m_left1,m_right1);
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
  
  public void stop() {
    drive.stopMotor();
  }
  
  /**
   * @param rightSide Gets encoder position from right side if true, otherwise left side.
   * @return Total distance driven in meters, as measured by TalonFX encoders.
   */
  public double getPosition(boolean rightSide) {
    if(rightSide) {
      return m_right1.getSelectedSensorPosition() * Constants.driveGearing * (Math.PI/180.0) * 0.1524;
    } else {
      return m_left1.getSelectedSensorPosition() * Constants.driveGearing * (Math.PI/180.0) * 0.1524;
    }
  }
  
  public void setZero() {
    m_right1.setSelectedSensorPosition(0);
    m_left1.setSelectedSensorPosition(0);
  }

  public double getOutput(boolean rightSide) {
    if(rightSide) {
      return m_right1.get();
    } else {
      return m_left1.get();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
