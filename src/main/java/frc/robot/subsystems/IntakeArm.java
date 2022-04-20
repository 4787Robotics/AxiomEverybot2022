// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;

public class IntakeArm extends SubsystemBase {
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private MotorController intakeMotor;
  private SparkMaxPIDController PID;
  private double kP, kI, kD; // pid gains
  private double p, i, d; // inputted values from SmartDashboard

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    armMotor = new CANSparkMax(Constants.motor_arm, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();
    intakeMotor = new WPI_TalonSRX(Constants.motor_intake);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(35);
    armEncoder.setPositionConversionFactor(360.0 * Constants.armGearing);
    armEncoder.setVelocityConversionFactor(360.0 * Constants.armGearing / 60.0);
    this.setEncoder(0);

    PID = armMotor.getPIDController();
    PID.setP(kP);
    PID.setI(kI);
    PID.setD(kD);
    PID.setFF(0);
    PID.setIZone(0);
    PID.setOutputRange(-1.0, 1.0);
    SmartDashboard.putNumber("Arm P gain",kP);
    SmartDashboard.putNumber("Arm I gain",kI);
    SmartDashboard.putNumber("Arm D gain",kD);
  }

  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void positionSetpoint(double setpoint) {
    PID.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }

  public void setEncoder(double value) {
    armEncoder.setPosition(value);
  }

  public double getPosition() {
    return armEncoder.getPosition(); //0-60 degrees range
  }
  
  public double getVelocity() {
    return armEncoder.getVelocity();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle",armEncoder.getPosition());
    SmartDashboard.putNumber("Arm Output",armMotor.get());
    SmartDashboard.putNumber("Intake Output",intakeMotor.get());
    SmartDashboard.putNumber("Arm Motor Temp", armMotor.getMotorTemperature());
    p = SmartDashboard.getNumber("Arm P gain",0);
    i = SmartDashboard.getNumber("Arm I gain",0);
    d = SmartDashboard.getNumber("Arm D gain",0);
    if(p != kP) {
      kP = p;
      PID.setP(kP);
    }
    if(i != kI) {
      kI = i;
      PID.setI(kI);
    }
    if(d != kD) {
      kD = d;
      PID.setD(kD);
    }
  }
}
