// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeArm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeCommand extends CommandBase {
  private IntakeArm intake;
  private DoubleSupplier ejectSpeedButNotBackwards;
  private DoubleSupplier armSetpoint;
  //NEEDS TUNING
  private PIDController armPID = new PIDController(0.006, 0, 0.001);
  
  /** Creates a new Intake. */
  public IntakeCommand(IntakeArm intake, DoubleSupplier ejectSpeedButNotBackwards, DoubleSupplier armSetpoint) {
    this.intake = intake;
    this.ejectSpeedButNotBackwards = ejectSpeedButNotBackwards;
    this.armSetpoint = armSetpoint;
    addRequirements(intake);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setEncoder(78);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setArmSpeed(armPID.calculate(intake.getPosition(), 78 - (83 * armSetpoint.getAsDouble())));
    if(intake.getPosition() <= 5) {
      intake.setIntakeSpeed(-1.0);
    } else {
      intake.setIntakeSpeed(ejectSpeedButNotBackwards.getAsDouble());
    }
    SmartDashboard.putNumber("Arm Angle", intake.getPosition());
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
