package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private DriveTrain driveTrain;
    private DoubleSupplier throttle;
    private DoubleSupplier steer;
    private BooleanSupplier slow;

    public DriveCommand(DriveTrain driveTrain, DoubleSupplier throttle, DoubleSupplier steer, BooleanSupplier slow) {
        this.driveTrain = driveTrain;
        this.throttle = throttle;
        this.steer = steer;
        this.slow = slow;
        addRequirements(driveTrain);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(slow.getAsBoolean()) {
            driveTrain.manualDrive(throttle.getAsDouble(), steer.getAsDouble(), 0.2, 0.2, true);
        } else {
            driveTrain.manualDrive(throttle.getAsDouble(), steer.getAsDouble(), 0.4, 0.4, true);
        }
        SmartDashboard.putNumber("Left Position", driveTrain.getPosition(false));
        SmartDashboard.putNumber("Right Position", driveTrain.getPosition(true));
        SmartDashboard.putNumber("Left Output", driveTrain.getOutput(false));
        SmartDashboard.putNumber("Right Output", driveTrain.getOutput(true));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
    return false;
    }

}
