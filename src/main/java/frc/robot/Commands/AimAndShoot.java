package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AimAndShoot extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private DriveSubsystem driveSubsystem;
    private double tx;
    private PIDController pid;

    /**
     * Turns to aim at the speaker and shoot
     */
    public AimAndShoot(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        //pid = new PIDController(LimelightConstants.kP, LimelightConstants.kI, LimelightConstants.kD);
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        tx = LimelightHelpers.getTX("limelight");
        while(LimelightConstants.kLowerTolerance > tx || tx > LimelightConstants.kUpperTolerance) {
            double aprilTagID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-1);
            SmartDashboard.putNumber("debugTx", tx);
            //if in tolerance
            if (aprilTagID == 5)
            {      // positive is to the right
                if (LimelightConstants.kLowerTolerance > tx){   //if too far beneath tolerance
                    // turn right
                    driveSubsystem.drive(0, 0, 5, true, true, false);
                }
                else if (tx > LimelightConstants.kUpperTolerance){      //if too far above tolerance
                    // turn left
                    driveSubsystem.drive(0, 0, -5, true, true, false);
                }
            }
            tx = LimelightHelpers.getTX("limelight");
        }
        // stop the robot
        // shoot!!!
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true, true, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}