package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AprilTagHeights;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.RollersSubsystem;

public class AimAndRev extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private DriveSubsystem driveSubsystem;
    private RollersSubsystem rollersSubsystem;
    private ArmSubsystem armSubsystem;
    private LedSubsystem ledSubsystem;
    private XboxController xboxController;
    private double tx;
    private double targetHeading;
    private double targetDistance;
    private double targetX;
    private double targetY;
    private double shootPosition;
    private boolean targetFound = false;
    private DigitalInput noteLimitSwitch;
    private double revSpeed;

    /**
     * Turns to aim at the speaker and shoot
     */
    public AimAndRev(DriveSubsystem driveSubsystem, RollersSubsystem rollersSubsystem, ArmSubsystem armSubsystem, LedSubsystem ledSubsystem, XboxController driveController, DigitalInput noteLimitSwitch) {
        this.driveSubsystem = driveSubsystem;
        this.rollersSubsystem = rollersSubsystem;
        this.armSubsystem = armSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.shootPosition = ArmConstants.speakerPos;
        xboxController = driveController;
        this.noteLimitSwitch = noteLimitSwitch;
        this.revSpeed = 0.5;

        addRequirements(driveSubsystem, rollersSubsystem, armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //rollersSubsystem.moveRoller(RollerConstants.outputRollerSpeed);
        targetFound = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        // Calulated the target angle (radians) from the coordinates of the target and the coordinates of the robot

        boolean hasTarget = LimelightHelpers.getTV(LimelightConstants.limelightAprilHostName);
        SmartDashboard.putBoolean("TV test", LimelightHelpers.getTV(LimelightConstants.limelightAprilHostName));
        double id = LimelightHelpers.getFiducialID(LimelightConstants.limelightAprilHostName);
        double driveX = driveSubsystem.getPose().getX();
        double driveY = driveSubsystem.getPose().getY();
        double driveHeading = driveSubsystem.getHeading();
        // if an april tag was seen
        if (hasTarget && (id == 4 || id == 7)){
            tx = -LimelightHelpers.getTX(LimelightConstants.limelightAprilHostName);
            SmartDashboard.putNumber("txAimAndRev", tx);
            double ty = LimelightHelpers.getTY(LimelightConstants.limelightAprilHostName);
            targetHeading = (tx + driveHeading) * Math.PI / 180;

            // d = (h2-h1) / tan(a1+a2)
            targetDistance = (AprilTagHeights.speaker - LimelightConstants.speakerLimelightHeight) / Math.tan((LimelightConstants.speakerLimelightAngle + ty) * Math.PI / 180);
            SmartDashboard.putNumber("wall distance", targetDistance);

            // adjust for limelight offset from the center of the robot
            double limelightX = driveX + LimelightConstants.xOffset * Math.sin(driveHeading) + LimelightConstants.yOffset * Math.cos(driveHeading);
            double limelightY = driveY + LimelightConstants.xOffset * Math.cos(driveHeading) + LimelightConstants.yOffset * Math.sin(driveHeading);
            
            targetX = limelightX + (targetDistance * Math.cos(targetHeading));
            targetY = limelightY + (targetDistance * Math.sin(targetHeading));
            targetFound = true;
        }

        if (targetFound) {
            double targetAngle = Math.atan2(targetY - driveY, targetX - driveX);
            // potentially get driver x and y input
            double xSpeed = -MathUtil.applyDeadband(xboxController.getLeftY(), OIConstants.kDriveDeadband);
            double ySpeed = -MathUtil.applyDeadband(xboxController.getLeftX(), OIConstants.kDriveDeadband);
            double rotSpeed = driveSubsystem.getDesiredHeadingSpeed(targetAngle);

            if (xSpeed == 0 && ySpeed == 0 && rotSpeed == 0) {
                driveSubsystem.setX();
            }

            driveSubsystem.drive(xSpeed, ySpeed, rotSpeed, true, true, false);
            
            // find the shooter position
            double td = targetDistance / 12;

            SmartDashboard.putNumber("td IntakeAndRollers", td);
            SmartDashboard.putNumber("targetX", targetX);
            SmartDashboard.putNumber("targetY", targetY);

            shootPosition = (-0.000002 * Math.pow(td, 3))
                    - (0.000180 * Math.pow(td, 2)) + (0.008983 * td) + 0.672893;
            // shootPosition = 0.703;
            revSpeed = MathUtil.clamp((targetDistance / 180), 0.75, 1);
        }else{
            shootPosition = 0.69;
            double xSpeed = -MathUtil.applyDeadband(xboxController.getLeftY(), OIConstants.kDriveDeadband);
            double ySpeed = -MathUtil.applyDeadband(xboxController.getLeftX(), OIConstants.kDriveDeadband);
            double rotSpeed = -MathUtil.applyDeadband(xboxController.getRightX(),
                    OIConstants.kDriveDeadband);
            driveSubsystem.drive(xSpeed, ySpeed, rotSpeed, true, true, true);
        }
        rollersSubsystem.moveRoller(revSpeed);
        armSubsystem.setPointArm(shootPosition);
        if (armSubsystem.getEncoderPosition() > shootPosition - 0.005 && armSubsystem.getEncoderPosition() < shootPosition + 0.005){
            ledSubsystem.changeColor(LEDConstants.green);
        }
        else{
            ledSubsystem.changeColor(LEDConstants.violet);
        }
        armSubsystem.resetIntegral();
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