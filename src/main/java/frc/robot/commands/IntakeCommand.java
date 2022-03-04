// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArm;

public class IntakeCommand extends CommandBase {

  //global variable (global scope)
  private IntakeArm intake;
  private DoubleSupplier forwardSpeed;
  private DoubleSupplier forwardSpeedButBackwards;
  
  /** Creates a new Intake. */
  public IntakeCommand(IntakeArm intake, DoubleSupplier forwardSpeed, DoubleSupplier forwardSpeedButBackwards) {
    this.intake = intake;
    this.forwardSpeed = forwardSpeed;
    this.forwardSpeedButBackwards = forwardSpeedButBackwards;
    addRequirements(intake);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeSpeed(forwardSpeed.getAsDouble() - forwardSpeedButBackwards.getAsDouble());
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
