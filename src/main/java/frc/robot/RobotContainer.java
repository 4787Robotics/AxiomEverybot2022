// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.commands.FindBall;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  WPI_VictorSPX m_left1 = new WPI_VictorSPX(Constants.motor_left1);
  WPI_VictorSPX m_left2 = new WPI_VictorSPX(Constants.motor_left2);
  WPI_VictorSPX m_right1 = new WPI_VictorSPX(Constants.motor_right1);
  WPI_VictorSPX m_right2 = new WPI_VictorSPX(Constants.motor_right2);
  private final DriveTrain driveTrain = new DriveTrain(new WPI_VictorSPX[]{m_left1,m_left2},new WPI_VictorSPX[]{m_right1,m_right2},true);
  private final XboxController controller = new XboxController(0);
  
  FindBall findBall = new FindBall(driveTrain);
  DriveCommand drive = new DriveCommand(driveTrain, controller);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  
  private void configureButtonBindings() {
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous mode
   */
  public Command getAutonomousCommand() {
    return findBall;
  }
  
  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   * 
   * @return the command to run in teleoperated mode
   */
  public Command getTeleopCommand() {
    return drive;
  }


}