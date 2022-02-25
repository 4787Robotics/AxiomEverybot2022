// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;

public class IntakeArm extends SubsystemBase {
  private MotorController armMotor;
  private Encoder armEncoder;
  private MotorController intakeMotor;
  private PIDController armPID;

  /** Creates a new IntakeArm. */
  public IntakeArm(MotorController arm, MotorController intake, Encoder encoder) {
    armMotor = arm;
    intakeMotor = intake;
    armEncoder = encoder;
    armEncoder.reset();
  }

  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public double getPosition() {
    return armEncoder.getDistance();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
