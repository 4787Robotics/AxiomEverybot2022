// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX m_left1;
  private WPI_TalonFX m_right1;
  private WPI_TalonFX m_left2;
  private WPI_TalonFX m_right2;
  
  private DifferentialDrive drive;
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_left1 = new WPI_TalonFX(Constants.motor_left1);
    m_left2 = new WPI_TalonFX(Constants.motor_left2);
    m_right1 = new WPI_TalonFX(Constants.motor_right1);
    m_right2 = new WPI_TalonFX(Constants.motor_right2);

    m_left1.setInverted(TalonFXInvertType.Clockwise);
    m_left2.setInverted(TalonFXInvertType.Clockwise);
    m_left2.follow(m_left1);
    m_right2.follow(m_right1);

    drive = new DifferentialDrive(m_left1,m_right1);
  }

  /**
   * arcadeDrive based teleoperated control. Left stick throttle, right stick turn.
   * @param controller XboxController for teleop.
   * @param maxSpeed Max speed [0.0..1.0].
   * @param maxTurnSpeed Max turn speed [0.0..1.0].
   * @param squareInputs If set, squares inputs for low speed precision.
   */
  public void manualDrive(double throttle, double steer, double maxSpeed, double maxTurnSpeed, boolean squareInputs) {
    drive.arcadeDrive(Math.sqrt(maxSpeed)*throttle, Math.sqrt(maxTurnSpeed)*steer, squareInputs);
  }

  /**
   * Drives and turns at a set speed.
   * @param speed [-1.0..1.0].
   * @param turnSpeed [-1.0..1.0].
   */
  public void autonomousDrive(double speed, double turnSpeed) {
    drive.arcadeDrive(speed,turnSpeed);
  }
  
  public void stop() {
    drive.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
