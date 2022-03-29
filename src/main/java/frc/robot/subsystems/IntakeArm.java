// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class IntakeArm extends SubsystemBase {
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private MotorController intakeMotorTop;
  private MotorController intakeMotorBottom;

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    armMotor = new CANSparkMax(Constants.motor_arm, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();
    intakeMotorTop = new WPI_TalonSRX(Constants.motor_intakeTop);
    intakeMotorBottom = new WPI_TalonSRX(Constants.motor_intakeBottom);
    this.setEncoder(0);
  }

  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotorTop.set(speed);
    intakeMotorBottom.set(-speed);
  }

  public void setEncoder(double value) {
    armEncoder.setPosition(value / (360.0 * Constants.armGearing));
  }

  public double getPosition() {
    return armEncoder.getPosition() * 360.0 * Constants.armGearing; //0-60 degrees range
  }
  
  public double getVelocity() {
    return armEncoder.getVelocity() * 360.0 * Constants.armGearing;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
