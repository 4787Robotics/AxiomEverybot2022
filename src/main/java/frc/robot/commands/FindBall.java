package frc.robot.commands;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class FindBall extends CommandBase {
    private DriveTrain drive;
    public FindBall(DriveTrain driveTrain) {
        
        drive = driveTrain;
        addRequirements(driveTrain);
    }
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry ta = table.getEntry("ta");
    public void intialize() {
        super.initialize();
    }
    
    public void execute() {
        if(tv.getDouble(0.0) == 0) {
            drive.autonomousDrive(0,0.4);
        } else if(tx.getDouble(0.0) != 0) {
            drive.autonomousDrive(0,tx.getDouble(0.0) / 27);
        }
        /*if(ta.getDouble(0.0) < 0.70) {
            drive.autonomousDrive(0.3,0);
        }*/
    }
    public void end(boolean interrupted) {
    
    }
    
    public boolean isFinished() {
        return false;
    }
}
