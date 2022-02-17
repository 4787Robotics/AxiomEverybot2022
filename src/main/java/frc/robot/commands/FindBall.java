package frc.robot.commands;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DriveTrain;

public class FindBall extends CommandBase {
    private DriveTrain drive;
    private PIDController pid = new PIDController(-0.036,0,-0.005);
    private double turnValue = 0;
    private double driveValue = 0;
    public FindBall(DriveTrain driveTrain) {
        
        drive = driveTrain;
        addRequirements(driveTrain);
    }
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx"); // target's horizontal degrees from center of camera
    NetworkTableEntry ty = table.getEntry("ty"); // vertical degrees
    NetworkTableEntry tv = table.getEntry("tv"); // 0 if no target, 1 if target
    NetworkTableEntry ta = table.getEntry("ta"); // area of target (in % of screen)
    public void intialize() {
        super.initialize();
    }
    
    public void execute() {
        //cool PID code
        if(tv.getDouble(0.0) == 1) { // if see ball
            turnValue = pid.calculate(tx.getDouble(0.0),0);
            if(ta.getDouble(0.0) < 15) { // if ball far away
                driveValue = 0.6-(ta.getDouble(0.0)/167);
            } else { // reached ball
                driveValue = 0;
            }
            // simulataneous turn + drive toward target
            drive.autonomousDrive(driveValue,turnValue);
        } else {
            drive.autonomousDrive(0, 0.4);
        }

        //old dumb non-PID code
        /*if(tv.getDouble(0.0) == 0) {
            drive.autonomousDrive(0,0.4);
        } else if(tx.getDouble(0.0) < -5) {
            drive.autonomousDrive(0,-0.5);
        } else if(tx.getDouble(0.0) > 5) {
            drive.autonomousDrive(0,0.5);
        } else if(ta.getDouble(0.0) < 25) {
            drive.autonomousDrive(0.7,0);
        }*/
    }
    public void end(boolean interrupted) {
    
    }
    
    public boolean isFinished() {
        return false;
    }
}
