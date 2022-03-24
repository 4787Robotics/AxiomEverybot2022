// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;

public class AutoDrive extends CommandBase {
  private DriveTrain driveTrain;
  private double distance;
  private boolean turn;
  private double positionLeft;
  private double positionRight;
  //NEEDS TUNING
  private PIDController drivePID = new PIDController(0, 0, 0);

  /** Creates a new AutoDrive. */
  public AutoDrive(DriveTrain driveTrain, double distance, boolean turn) {
    this.driveTrain = driveTrain;
    if(turn) {
      this.distance = Constants.turnRadius * distance * Math.PI / 180.0;
    } else {
      this.distance = distance;
    }
    this.turn = turn;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    positionLeft = driveTrain.getPosition(false);
    positionRight = driveTrain.getPosition(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driveTrain.getPosition(false) < distance + positionLeft) {
      if(turn) {
        driveTrain.autonomousTank(
          drivePID.calculate(driveTrain.getPosition(false), distance + positionLeft),
          drivePID.calculate(driveTrain.getPosition(true), -distance + positionRight)
        );
      } else {
        driveTrain.autonomousTank(
          drivePID.calculate(driveTrain.getPosition(false), distance + positionLeft),
          drivePID.calculate(driveTrain.getPosition(true), distance + positionRight)
        );
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.autonomousTank(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTrain.getPosition(false) >= distance + positionLeft;
  }
}
