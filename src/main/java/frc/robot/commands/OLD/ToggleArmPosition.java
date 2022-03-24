// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.OLD;

import frc.robot.subsystems.IntakeArm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NO LONGER USING THIS COMMAND

public class ToggleArmPosition extends CommandBase {
  private IntakeArm intake;
  private ProfiledPIDController armPID;
  private boolean intakeUp = true; 

  public ToggleArmPosition(IntakeArm intake) {
    this.intake = intake;
    addRequirements(intake);
    armPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(5,10));
    
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
    SmartDashboard.putNumber("Arm Position", intake.getPosition());
    if(intake.getPosition() >= 30) {
      intakeUp = true;
    } else {
      intakeUp = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("ARM: " + intakeUp);
    //measured in degrees
    if(intakeUp) {
      intake.setArmSpeed(armPID.calculate(intake.getPosition(), 0));
    } else {
      intake.setArmSpeed(armPID.calculate(intake.getPosition(), 60));
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setArmSpeed(0);
  }

  // Returns true when the command should end.

  public boolean isFinished() {
    /*if(intakeUp) { // check if arm has gone down
      return (Math.abs(intake.getPosition()) <= 1);
    } else { // check if arm has been raised
      return (Math.abs(intake.getPosition() - 60) <= 1);
    }*/
    return false;
  }
  
  

}