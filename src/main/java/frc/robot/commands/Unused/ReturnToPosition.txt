// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Unused;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ReturnToPosition extends CommandBase {
  private DriveTrain driveTrain;
  private double distanceLeft;
  private double distanceRight;
  //NEEDS TUNING
  private ProfiledPIDController PID = new ProfiledPIDController(0.1,0,0,new TrapezoidProfile.Constraints(1,1));
  
  /** Creates a new DriveDistance. */
  public ReturnToPosition(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    distanceLeft = driveTrain.getPosition(false);
    distanceRight = driveTrain.getPosition(true);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Started driving backwards");
    driveTrain.setZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("driving");
    driveTrain.autonomousTank(
      PID.calculate(driveTrain.getPosition(false),-distanceLeft),
      PID.calculate(driveTrain.getPosition(true),-distanceRight)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.autonomousTank(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (driveTrain.getPosition(true) == -distanceRight) && (driveTrain.getPosition(false) == -distanceLeft);
  }
}
