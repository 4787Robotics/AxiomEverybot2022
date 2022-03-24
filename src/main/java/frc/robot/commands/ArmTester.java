// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeArm;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmTester extends CommandBase {
  private IntakeArm intake;
  private XboxController controller;
  /** Creates a new ArmTester. */
  public ArmTester(IntakeArm intake, XboxController controller) {
    this.intake = intake;
    this.controller = controller;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setArmSpeed((controller.getLeftTriggerAxis() - controller.getRightTriggerAxis())*0.3);
    if(controller.getLeftTriggerAxis()==0 && controller.getRightTriggerAxis()==0 && intake.getPosition() < 7) {
      intake.setArmSpeed(-0.1);
    }
    if(controller.getRawButton(6)) {
      intake.setIntakeSpeed(1);
    } else if(controller.getRawButton(5)) {
      intake.setIntakeSpeed(-0.5);
    } else {
      intake.setIntakeSpeed(0);
    }
    SmartDashboard.putNumber("Arm Angle",intake.getPosition());
    SmartDashboard.putNumber("Arm Speed",intake.getVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
