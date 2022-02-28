package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    private DriveTrain driveTrain;
    private DoubleSupplier throttle;
    private DoubleSupplier steer;

    public DriveCommand(DriveTrain driveTrain, DoubleSupplier throttle, DoubleSupplier steer) {
        this.driveTrain = driveTrain;
        this.throttle = throttle;
        this.steer = steer;
        addRequirements(driveTrain);
    }
    
    public void initialize() {}

    public void execute() {
        driveTrain.manualDrive(throttle.getAsDouble(), steer.getAsDouble(), 0.7, 0.7, true);
    }
    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
    return false;
    }

}
