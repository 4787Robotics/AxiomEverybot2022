package frc.robot.commands;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FindBall {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    public void intialize() {

    }
    
    public void execute() {
        if(tx.getDouble(0.0) > 0) {
            //turn right
        } else if(tx.getDouble(0.0) < 0) {
            //turn left
        } else {
            //stop
        }
    }
    public void end(boolean interrupted) {
    
    }
    
    public boolean isFinished() {
        return false;
    }
}
