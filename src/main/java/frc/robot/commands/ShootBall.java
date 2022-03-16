// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeArm;

public class ShootBall extends CommandBase {
  private DriveTrain driveTrain;
  private IntakeArm intakeArm;

  // Limelight variables
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx"); // target's horizontal degrees from center of camera
  NetworkTableEntry ty = table.getEntry("ty"); // vertical degrees
  NetworkTableEntry tv = table.getEntry("tv"); // 0 if no target, 1 if target
  NetworkTableEntry ta = table.getEntry("ta"); // area of target (in % of screen)

  //Getting in Range
  private double kpDistance = -0.1; //placeholder number for now, will need tuning

  //GetDistanceVariables
  double offsetAngle;
  double oppositeHeight;
  double trigTanValue;
  double distance;

  /** Creates a new ShootBall. */
  public ShootBall(IntakeArm intakeArm, DriveTrain driveTrain) {
    addRequirements(intakeArm, driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(getDistance()); //0.3 meters off (margin of error?!?!?!?!?!?)
    if (driveTrain.getPosition() < (getDistance() * 1.00)) {
      driveTrain.autonomousDrive(0.15, 0);
    }
    else {

    }
  }
    

  public double getDistance() {
    offsetAngle = ty.getDouble(0.0);
    oppositeHeight = Constants.hubHeight - Constants.limelightHeight;
    trigTanValue = Math.tan((offsetAngle + Constants.limelightMountAngle) * (Math.PI/180));
    distance = oppositeHeight / trigTanValue;

    return distance;
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
