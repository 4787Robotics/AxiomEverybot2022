// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeArm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

public class ToggleArmPosition extends CommandBase {
  private IntakeArm intake;
  private PIDController armPID;
  private boolean intakeUp = true; 

  public ToggleArmPosition(IntakeArm intake) {
    this.intake = intake;
    addRequirements(intake);
    armPID = new PIDController(0, 0, 0);
    
    /**
     * @param mV max Velocity
     * @param mA max Accelration
     * @param kDt Time in Motion? idk man
     * PID = new ProfiledPIDController(kP, kV, kD, new TrapezoidProfile.constraints(mV, mA), kDt);
     * 
     * if(!intakeUp)
     *    PID.setGoal(position value OR TrapezoidProfile.State, if nonzero velocity is desired);
     * else 
     *    PID.setGoal(0);
     * 
     * intake.setArmSpeed(PID.calculate(intake.getPosition()))
     */
  }

  @Override
  public void initialize() {
    if(intake.getPosition() >= 30) {
      intakeUp = true;
    } else {
      intakeUp = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //measured in degrees
    if(intakeUp) {
      intake.setArmSpeed(armPID.calculate(intake.getPosition(), 0));
    } else {
      intake.setArmSpeed(armPID.calculate(intake.getPosition(), 60));
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intakeUp) { // check if arm has gone down
      return (Math.abs(intake.getPosition()) <= 1) && (Math.abs(intake.getVelocity()) <= 1);
    } else { // check if arm has been raised
      return (Math.abs(intake.getPosition() - 60) <= 1) && (Math.abs(intake.getVelocity()) <= 1);
    }
  }
}
