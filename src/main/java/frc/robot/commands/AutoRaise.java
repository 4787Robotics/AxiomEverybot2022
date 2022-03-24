// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArm;

public class AutoRaise extends CommandBase {
  private IntakeArm intakeArm;
  private boolean intakeUp;
  //NEEDS TUNING
  private ProfiledPIDController armPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(5,5));

  /** Creates a new AutoRaise. */
  public AutoRaise(IntakeArm intakeArm, boolean intakeUp) {
    this.intakeUp = intakeUp;
    this.intakeArm = intakeArm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intakeUp) {
      intakeArm.setArmSpeed(armPID.calculate(intakeArm.getPosition(), 60));
    } else if(intakeUp) {
      intakeArm.setArmSpeed(armPID.calculate(intakeArm.getPosition(), 0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeArm.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intakeUp && intakeArm.getPosition() >= 60) || (!intakeUp && intakeArm.getPosition() <= 1);
  }
}
