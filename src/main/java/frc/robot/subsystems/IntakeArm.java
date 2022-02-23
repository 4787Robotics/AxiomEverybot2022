// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class IntakeArm extends SubsystemBase {
  private MotorController armMotor;
  private MotorController intakeMotor;

  /** Creates a new IntakeArm. */
  public IntakeArm(MotorController arm, MotorController intake) {
    armMotor = arm;
    intakeMotor = intake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
