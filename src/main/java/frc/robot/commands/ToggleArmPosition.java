// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeArm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

public class ToggleArmPosition extends CommandBase {
  private IntakeArm intake;
  private PIDController armPID;

  /** Creates a new ToggleArmPosition. */
  public ToggleArmPosition(IntakeArm intake) {
    this.intake = intake;
    addRequirements(intake);
    armPID = new PIDController(0, 0, 0);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //measured in degrees
    if(intake.getPosition() != 60) 
      intake.setArmSpeed(armPID.calculate(intake.getPosition(), 60));
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
