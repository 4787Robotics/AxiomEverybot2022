// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeArm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeCommand extends CommandBase {
  private IntakeArm intake;
  private DoubleSupplier forwardSpeed;
  private DoubleSupplier forwardSpeedButBackwards;
  private BooleanSupplier raiseArm;
  private BooleanSupplier lowerArm;
  private ProfiledPIDController armPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(5,5));
  
  /** Creates a new Intake. */
  public IntakeCommand(IntakeArm intake, DoubleSupplier forwardSpeed, DoubleSupplier forwardSpeedButBackwards, BooleanSupplier raiseArm, BooleanSupplier lowerArm) {
    this.intake = intake;
    this.forwardSpeed = forwardSpeed;
    this.forwardSpeedButBackwards = forwardSpeedButBackwards;
    this.raiseArm = raiseArm;
    this.lowerArm = lowerArm;
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
