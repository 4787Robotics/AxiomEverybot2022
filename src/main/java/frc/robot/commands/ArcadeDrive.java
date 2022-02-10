package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDrive extends CommandBase {
    private final DriveTrain driveTrain;
    private final XboxController control = new XboxController(0);

    public ArcadeDrive(DriveTrain drivetrain) {
        driveTrain = drivetrain;
        addRequirements(driveTrain);
    }
    /*
    public ArcadeDrive(DriveTrain driveTrain2, Object object, Object object2) {
    }
    */

    public void initialize() {

    }

    public void execute() {
        driveTrain.manualDrive(control, 2, 3, false);
    }
    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
    return false;
    }

}
