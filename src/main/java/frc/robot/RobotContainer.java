// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  XboxController controller = new XboxController(0);
  Joystick joystick = new Joystick(1);

  DriveTrain driveTrain = new DriveTrain();
  IntakeArm intake = new IntakeArm();
  Climber climber = new Climber();

  ParallelCommandGroup teleopGroup = new ParallelCommandGroup(
    new DriveCommand(driveTrain, ()-> -controller.getLeftY(), ()-> controller.getRightX(), ()-> controller.getRawButton(1)),
    /*new IntakeCommand(
      intake,
      ()-> controller.getRightTriggerAxis(),
      ()-> controller.getLeftTriggerAxis()
    ),*/
    new ArmTester(intake, controller),
    new ExtendClimber(climber, () -> joystick.getY())
  );
  Autonomous autonomous = new Autonomous(driveTrain, intake);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {}
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous mode
   */
  public Command getAutonomousCommand() {
    return autonomous;
  }
  
  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   * 
   * @return the command to run in teleoperated mode
   */
  public Command getTeleopCommand() {
    return teleopGroup;
  }
}