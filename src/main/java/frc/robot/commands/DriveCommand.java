package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    private DriveTrain driveTrain;
    private XboxController control;

    public DriveCommand(DriveTrain drivetrain, XboxController controller) {
        driveTrain = drivetrain;
        control = controller;
        addRequirements(drivetrain);
    }
    
    public void initialize() {}

    public void execute() {
        driveTrain.manualDrive(control, 0.5, 0.5, true);
    }
    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
    return false;
    }

}
