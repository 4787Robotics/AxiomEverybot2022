// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  private DriveTrain driveTrain;
  private IntakeArm intake;
  /** Creates a new Autonomous. */

  public Autonomous(DriveTrain driveTrain, IntakeArm intake) {
    this.driveTrain = driveTrain;
    this.intake = intake;
    
    addCommands(
      new ParallelRaceGroup(new AutoShoot(intake, 1), new WaitCommand(2)), //shoots the ball
      new AutoDrive(driveTrain, 180, true), //turns 180 degrees
      new AutoRaise(intake, false), //lowers the arm
      new ParallelRaceGroup(new AutoShoot(intake, -1), new FindBall(driveTrain, true)), //finds the ball and intakes it
      new AutoRaise(intake, true), //raises the arm
      new ReturnToPosition(driveTrain),
      new ParallelRaceGroup(new AutoShoot(intake,1), new WaitCommand(2)), //shoots the ball
      new AutoDrive(driveTrain, 180, true),
      new AutoDrive(driveTrain, 1.5, false)
    );
  }
}
