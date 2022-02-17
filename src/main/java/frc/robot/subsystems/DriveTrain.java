// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.XboxController;

public class DriveTrain extends SubsystemBase {
  private MotorControllerGroup m_left;
  private MotorControllerGroup m_right;
  private DifferentialDrive drive;
  
  /** Creates a new DriveTrain. */
  public DriveTrain(MotorController[] motorsLeft, MotorController[] motorsRight, boolean invertRightSide) {
    m_left = new MotorControllerGroup(motorsLeft);
    m_right = new MotorControllerGroup(motorsRight);
    m_right.setInverted(invertRightSide);
    drive = new DifferentialDrive(m_left,m_right);
  }

  /**
   * arcadeDrive based teleoperated control. Left stick throttle, right stick turn.
   * @param controller XboxController for teleop.
   * @param maxSpeed Max speed [0.0..1.0].
   * @param maxTurnSpeed Max turn speed [0.0..1.0].
   * @param squareInputs If set, squares inputs for low speed precision.
   */
  public void manualDrive(XboxController controller, double maxSpeed, double maxTurnSpeed, boolean squareInputs) {
    drive.arcadeDrive(-maxSpeed*controller.getLeftY(), maxTurnSpeed*controller.getRightX(), squareInputs);
  }

  /**
   * curvatureDrive based teleoperated control. Triggers throttle, left stick to turn.
   * @param controller XboxController for teleop.
   * @param maxSpeed Max speed [0.0..1.0].
   * @param maxTurnSpeed Max turn speed [0.0..1.0].
   */
  public void GTADrive(XboxController controller, double maxSpeed, double maxTurnSpeed) {
    drive.curvatureDrive(maxSpeed*(controller.getRightTriggerAxis()-controller.getLeftTriggerAxis()), controller.getLeftX(), false);
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
