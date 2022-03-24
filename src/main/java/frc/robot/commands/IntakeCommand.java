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
  private DoubleSupplier ejectSpeedButNotBackwards;
  private BooleanSupplier lowerArm;
  //NEEDS TUNING
  private ProfiledPIDController armPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(5,5));
  
  /** Creates a new Intake. */
  public IntakeCommand(IntakeArm intake, DoubleSupplier ejectSpeedButNotBackwards, BooleanSupplier lowerArm) {
    this.intake = intake;
    this.ejectSpeedButNotBackwards = ejectSpeedButNotBackwards;
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
    if(lowerArm.getAsBoolean()) {
      intake.setArmSpeed(armPID.calculate(intake.getPosition(), 0));
      intake.setIntakeSpeed(1.0);
    } else {
      intake.setArmSpeed(armPID.calculate(intake.getPosition(), 60));
      intake.setIntakeSpeed(-ejectSpeedButNotBackwards.getAsDouble());
    }
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
