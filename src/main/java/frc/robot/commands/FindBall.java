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
        if(tv.getDouble(0.0) == 0) {
            //turn until there is a ball
        }
        
        if(tx.getDouble(0.0) > 0) {
            //turn right
        } else if(tx.getDouble(0.0) < 0) {
            //turn left
        }

        if(ta.getDouble(0.0) < 0.70) {
            //moveforward
        }
    }
    public void end(boolean interrupted) {
    
    }
    
    public boolean isFinished() {
        return false;
    }
}
