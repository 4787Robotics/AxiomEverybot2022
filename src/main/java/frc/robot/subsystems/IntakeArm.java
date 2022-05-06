// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
    // make sure to set motortype for spark max, if it's incorrect it may damage the NEOs
    armMotor = new CANSparkMax(Constants.motor_arm, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();
    intakeMotor = new WPI_TalonSRX(Constants.motor_intake);
    armMotor.setIdleMode(IdleMode.kBrake); // equivalent to setNeutralMode from CTRE Phoenix library (the one for Falcon 500)
    armMotor.setSmartCurrentLimit(35); // mitigates current spikes so the NEOs don't fry themselves like they did at Midwest
    armEncoder.setPositionConversionFactor(360.0 * Constants.armGearing); // for encoder
    armEncoder.setVelocityConversionFactor(360.0 * Constants.armGearing / 60.0); // ^
    this.setEncoder(0);

    /* all unused but apparently you can run PID directly on the Spark Max for better response and accuracy
      (as opposed to using WPI's PID that runs on the RoboRIO) */
    PID = armMotor.getPIDController();
    PID.setP(kP);
    PID.setI(kI);
    PID.setD(kD);
    PID.setFF(0);
    PID.setIZone(0);
    PID.setOutputRange(-1.0, 1.0);
    SmartDashboard.putNumber("Arm P gain",kP); // Adds input boxes to SmartDashboard so you can
    SmartDashboard.putNumber("Arm I gain",kI); // adjust PID gains while code is running.
    SmartDashboard.putNumber("Arm D gain",kD); // Values are read in periodic().
  }

  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void positionSetpoint(double setpoint) {
    PID.setReference(setpoint, CANSparkMax.ControlType.kPosition); // for PID
  }

  public void setEncoder(double value) {
    armEncoder.setPosition(value);
  }

  public double getPosition() {
    return armEncoder.getPosition(); // degrees
  }
  
  public double getVelocity() {
    return armEncoder.getVelocity(); // in degrees/sec i think i dont remember
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle",armEncoder.getPosition());
    SmartDashboard.putNumber("Arm Output",armMotor.get());
    SmartDashboard.putNumber("Intake Output",intakeMotor.get());
    SmartDashboard.putNumber("Arm Motor Temp", armMotor.getMotorTemperature());

    // gets the inputs you can type into the SmartDashboard boxes
    p = SmartDashboard.getNumber("Arm P gain",0);
    i = SmartDashboard.getNumber("Arm I gain",0);
    d = SmartDashboard.getNumber("Arm D gain",0);
    if(p != kP) { // if you typed something into the box, changes it in code
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
