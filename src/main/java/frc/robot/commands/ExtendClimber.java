// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendClimber extends CommandBase {
  private Climber climber;
  private DoubleSupplier climbSpeed;
  private BooleanSupplier rightSideDown, rightSideUp, leftSideDown, leftSideUp;
  /** Creates a new ClimbTester. */
  public ExtendClimber(Climber climber, DoubleSupplier climbSpeed, BooleanSupplier rightSideDown, BooleanSupplier rightSideUp, BooleanSupplier leftSideDown, BooleanSupplier leftSideUp) {
    this.climber = climber;
    this.climbSpeed = climbSpeed;
    this.rightSideDown = rightSideDown;
    this.rightSideUp = rightSideUp;
    this.leftSideDown = leftSideDown;
    this.leftSideUp = leftSideUp;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(rightSideDown.getAsBoolean()) {
      climber.setSideSpeed(0.3, true);
    } else if(rightSideUp.getAsBoolean()) {
      climber.setSideSpeed(-0.3, true);
    } else {
      climber.setSideSpeed(climbSpeed.getAsDouble(), true);
    }

    if(leftSideDown.getAsBoolean()) {
      climber.setSideSpeed(0.3, false);
    } else if(leftSideUp.getAsBoolean()) {
      climber.setSideSpeed(-0.3, false);
    } else {
      climber.setSideSpeed(climbSpeed.getAsDouble(), false);
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
