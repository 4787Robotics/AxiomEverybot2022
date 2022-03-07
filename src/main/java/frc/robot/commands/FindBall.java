package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FindBall extends CommandBase {
    // Drive and PID variables
    private DriveTrain driveTrain;
    private PIDController drivePID = new PIDController(-0.036,0,-0.005); // OUTDATED CONSTANTS
    private double turnValue = 0;
    private double driveValue = 0;

    // Limelight variables
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx"); // target's horizontal degrees from center of camera
    NetworkTableEntry ty = table.getEntry("ty"); // vertical degrees
    NetworkTableEntry tv = table.getEntry("tv"); // 0 if no target, 1 if target
    NetworkTableEntry ta = table.getEntry("ta"); // area of target (in % of screen)
    private boolean pipeType; // toggle to red ball pipeline (true) or blue (false)

    /**
     * Creates a new FindBall command.
     * @param driveTrain The DriveTrain subsystem to be used.
     * @param pipeType Toggles Limelight pipeline for ball detection. true = red, false = blue
     */
    public FindBall(DriveTrain driveTrain, boolean pipeType) {
        this.driveTrain = driveTrain;
        this.pipeType = pipeType;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (pipeType) {
            table.getEntry("pipeline").setNumber(0);
        }
        else {
            table.getEntry("pipeline").setNumber(1);
        }

        if(tv.getDouble(0.0) == 1) { // if see ball
            turnValue = drivePID.calculate(tx.getDouble(0.0),0);
            if(ta.getDouble(0.0) < 15) { // if ball far away
                driveValue = 0.6-(ta.getDouble(0.0)/167);
            } else { // reached ball
                driveValue = 0;
            }
            // simulataneous turn + drive toward target
            driveTrain.autonomousDrive(driveValue,turnValue);
        } else {
            driveTrain.autonomousDrive(0, 0.4);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }


}
