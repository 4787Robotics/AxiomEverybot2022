// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.Unused.FindBall;
import edu.wpi.first.wpilibj.GenericHID;
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
  DriveTrain driveTrain = new DriveTrain();
  XboxController controller = new XboxController(0);

  IntakeArm intake = new IntakeArm();
  Climber climber = new Climber();

  FindBall findBall = new FindBall(driveTrain, true);
  //ShootBall shootBall = new ShootBall(intake, driveTrain);

  ParallelCommandGroup teleop = new ParallelCommandGroup(
    new DriveCommand(driveTrain, ()-> -controller.getLeftY(), ()-> controller.getRightX(), ()-> controller.getRawButton(1)),
    /*new IntakeCommand(
      intake,
      ()-> controller.getLeftTriggerAxis(),
      ()-> controller.getRawButton(Constants.lowerButton),
      ()-> controller.getRawButton(Constants.halfLowerButton)
    )*/
    //new ArmTester(intake, controller)
    new ExtendClimber(climber, () -> controller.getLeftTriggerAxis() - controller.getRightTriggerAxis())
  );
  Autonomous autonomous = new Autonomous(driveTrain, intake);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  /*private void configureButtonBindings() {
     screw this shit
    armPositionButton.whenPressed(new ParallelCommandGroup(new ToggleArmPosition(intake));
    shootBallButton.whileActiveContinuous(new DriveDistance(driveTrain));
    //QUICK NOTE: WE WANT TO CONFIGURE THE TYPE OF BUTTON FUNCTION THAT WE WANT TO USE
    // armPositionButton.whenPressed(command)
  }*/
  
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
    return teleop;
  }
}