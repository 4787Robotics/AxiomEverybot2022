// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.math.trajectory.Trajectory;
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
  private XboxController controller;
  private Joystick joystick;
  public static DriveTrain driveTrain = new DriveTrain();
  private IntakeArm intake;
  private Climber climber;
  private ParallelCommandGroup teleopCommand;
  private Autonomous autoCommand;
  //private static PathFollowingAutonomousCommand autoCommand = new PathFollowingAutonomousCommand();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    controller = new XboxController(0);
    joystick = new Joystick(1);

   
    intake = new IntakeArm();
    climber = new Climber();
    
    teleopCommand = new ParallelCommandGroup(
      new DriveCommand(driveTrain, ()-> -controller.getLeftY(), ()-> controller.getRightX(), ()-> controller.getRawButton(1)),
      /*new IntakeCommand(
        intake,
        ()-> controller.getRightTriggerAxis(),
        ()-> controller.getLeftTriggerAxis()
      ),*/
      new ArmTester(intake, controller),
      new ExtendClimber(climber, () -> 0.4*joystick.getY(), () -> joystick.getRawButton(3), () -> joystick.getRawButton(5), () -> joystick.getRawButton(4), () -> joystick.getRawButton(6))
    );
    autoCommand = new Autonomous(driveTrain, intake);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous mode
   */
  public Command getAutonomousCommand() {
    return autoCommand;
    //return autoCommand.createAutonomousCommand(trajectory);
  }
  
  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   * 
   * @return the command to run in teleoperated mode
   */
  public Command getTeleopCommand() {
    return teleopCommand;
  }

  public void resetOdometry() {
  //  autoCommand.resetOdometryInitialPose();
  }
}