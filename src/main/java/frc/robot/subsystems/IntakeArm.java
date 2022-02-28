// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeArm extends SubsystemBase {
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private MotorController intakeMotor;

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    armMotor = new CANSparkMax(Constants.motor_arm, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();
    armEncoder.setPosition(0);
    armEncoder.setPositionConversionFactor(Constants.armGearing);
    //intakeMotor = new ... ;
  }

  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public double getPosition() {
    return armEncoder.getPosition();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}