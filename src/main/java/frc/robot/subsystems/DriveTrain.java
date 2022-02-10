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
  public DriveTrain(MotorController[] motorsLeft, MotorController[] motorsRight) {
    m_left = new MotorControllerGroup(motorsLeft);
    m_right = new MotorControllerGroup(motorsRight);
    drive = new DifferentialDrive(m_left,m_right);
  }

  public void manualDrive(XboxController controller, double maxSpeed, double maxTurnSpeed, boolean squareInputs) {
    drive.arcadeDrive(maxSpeed*controller.getLeftX(), maxTurnSpeed*controller.getLeftY(), squareInputs);
  }
  
  public void stop() {
    drive.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
