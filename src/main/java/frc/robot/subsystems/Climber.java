// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Climber extends SubsystemBase {
  private WPI_TalonFX motorLeft, motorRight;

  /** Creates a new Climber. */
  public Climber() {
    motorLeft = new WPI_TalonFX(Constants.motor_climbLeft);
    motorRight = new WPI_TalonFX(Constants.motor_climbRight);
  }

  public void setSpeed(double speed) {
    motorLeft.set(speed);
    motorRight.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Output", motorLeft.get());
  }
}
