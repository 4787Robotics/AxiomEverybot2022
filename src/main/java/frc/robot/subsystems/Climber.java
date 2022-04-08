// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Climber extends SubsystemBase {
  private WPI_TalonFX motorLeft, motorRight;

  /** Creates a new Climber. */
  public Climber() {
    motorLeft = new WPI_TalonFX(Constants.motor_climbLeft);
    motorRight = new WPI_TalonFX(Constants.motor_climbRight);
    motorRight.setInverted(true);
    motorLeft.setNeutralMode(NeutralMode.Brake);
    motorRight.setNeutralMode(NeutralMode.Brake);
  }

  public void setSpeed(double speed) {
    motorLeft.set(speed);
    motorRight.set(speed);
  }
  
  public void setSideSpeed(double speed, boolean rightSide) {
    if(rightSide) {
      motorRight.set(speed);
    } else {
      motorLeft.set(speed);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Left Output", motorLeft.get());
    SmartDashboard.putNumber("Climber Right Output", motorRight.get());
  }
}
