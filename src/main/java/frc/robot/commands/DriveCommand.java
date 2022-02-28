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
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        driveTrain.manualDrive(throttle.getAsDouble(), steer.getAsDouble(), 0.7, 0.7, true);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
    return false;
    }

}
