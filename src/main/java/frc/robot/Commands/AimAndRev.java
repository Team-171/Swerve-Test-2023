package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AprilTagHeights;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RollersSubsystem;

public class AimAndRev extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private DriveSubsystem driveSubsystem;
    private RollersSubsystem rollersSubsystem;
    private ArmSubsystem armSubsystem;
    private XboxController xboxController;
    private double tx;
    private double targetHeading;
    private double targetDistance;
    private double targetX;
    private double targetY;
    private double shootPosition;
    private boolean targetFound = false;

    /**
     * Turns to aim at the speaker and shoot
     */
    public AimAndRev(DriveSubsystem driveSubsystem, RollersSubsystem rollersSubsystem, ArmSubsystem armSubsystem, XboxController driveController) {
        this.driveSubsystem = driveSubsystem;
        this.rollersSubsystem = rollersSubsystem;
        this.armSubsystem = armSubsystem;
        this.shootPosition = ArmConstants.speakerPos;
        xboxController = driveController;
        addRequirements(driveSubsystem, rollersSubsystem, armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rollersSubsystem.moveRoller(RollerConstants.outputRollerSpeed);
        targetFound = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        // Calulated the target angle (radians) from the coordinates of the target and the coordinates of the robot

        boolean hasTarget = LimelightHelpers.getTV(LimelightConstants.limelightAprilHostName);
        SmartDashboard.putBoolean("TV test", LimelightHelpers.getTV(LimelightConstants.limelightAprilHostName));
        double driveX = driveSubsystem.getPose().getX();
        double driveY = driveSubsystem.getPose().getY();
        double driveHeading = driveSubsystem.getHeading();
        // if an april tag was seen
        if (hasTarget){
            tx = -LimelightHelpers.getTX(LimelightConstants.limelightAprilHostName);
            double ty = LimelightHelpers.getTY(LimelightConstants.limelightAprilHostName);
            targetHeading = (tx + driveHeading) * Math.PI / 180;

            // d = (h2-h1) / tan(a1+a2)
            targetDistance = (AprilTagHeights.speaker - LimelightConstants.speakerLimelightHeight) / Math.tan((LimelightConstants.speakerLimelightAngle + ty) * Math.PI / 180);

            // adjust for limelight offset from the center of the robot
            double limelightX = driveX + LimelightConstants.xOffset * Math.sin(driveHeading) + LimelightConstants.yOffset * Math.cos(driveHeading);
            double limelightY = driveY + LimelightConstants.xOffset * Math.cos(driveHeading) + LimelightConstants.yOffset * Math.sin(driveHeading);
            
            targetX = limelightX + (targetDistance * Math.cos(targetHeading));
            targetY = limelightY + (targetDistance * Math.sin(targetHeading));
            SmartDashboard.putNumber("targetX", targetX);
            SmartDashboard.putNumber("targetY", targetY);
            targetFound = true;
        }

        if (targetFound) {
            double targetAngle = Math.atan2(targetY - driveY, targetX - driveX);
            SmartDashboard.putNumber("Target Angle", targetAngle);
            // potentially get driver x and y input
            double xSpeed = -MathUtil.applyDeadband(xboxController.getLeftY(), OIConstants.kDriveDeadband);
            double ySpeed = -MathUtil.applyDeadband(xboxController.getLeftY(), OIConstants.kDriveDeadband);

            driveSubsystem.drive(xSpeed, ySpeed, driveSubsystem.getDesiredHeadingSpeed(targetAngle), true, true, false);
            
            // find the shooter position
            double td = targetDistance / 12;

            SmartDashboard.putNumber("td IntakeAndRollers", td);

            shootPosition = (0.000019 * Math.pow(td, 3))
                    - (0.001047 * Math.pow(td, 2)) + (0.020010 * td) + 0.629581;
        }
        SmartDashboard.putNumber("arm encoder", armSubsystem.getEncoderPosition());
        armSubsystem.setPointArm(shootPosition);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true, true, false);
        rollersSubsystem.moveRoller(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // only ends when button is released
        return false;
    }
}