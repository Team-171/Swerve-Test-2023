package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTagHeights;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollersSubsystem;

public class IntakeAndRollers extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private RollersSubsystem rollersSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ArmSubsystem armSubsystem;
    private DriveSubsystem driveSubsystem;
    private double intakeSpeed;
    private double rollerSpeed;
    private double indexSpeed;
    private double armPosition;
    private boolean hasTarget;
    private double desiredHeading;
    private double tx;
    private boolean targetFound;
    private XboxController xboxController;
    private DigitalInput noteLimitSwitch;

    /**
     * Follows a given trajectory for autonomous.
     * 
     * @param trajectory Trajectory to follow
     * @param subsystem  Drive subsystem to drive the robot
     */
    public IntakeAndRollers(RollersSubsystem rollersSubsystem, IntakeSubsystem intakeSubsystem,
            ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem, XboxController xboxController, DigitalInput noteLimitSwitch, 
            double rollerSpeed, double indexSpeed, double intakeSpeed, double armPosition) {
        this.rollersSubsystem = rollersSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.rollerSpeed = rollerSpeed;
        this.indexSpeed = indexSpeed;
        this.intakeSpeed = intakeSpeed;
        this.armPosition = armPosition;
        this.xboxController = xboxController;
        this.noteLimitSwitch = noteLimitSwitch;
        

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(rollersSubsystem, intakeSubsystem, armSubsystem, driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        armSubsystem.setPointArm(armPosition);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (armSubsystem.getEncoderPosition() > armPosition - 0.05) {
            intakeSubsystem.runIntake(intakeSpeed);
            rollersSubsystem.index(indexSpeed);
            rollersSubsystem.moveRoller(rollerSpeed);
            armSubsystem.resetIntegral();
        }
        
        hasTarget = LimelightHelpers.getTV(LimelightConstants.limelightFloorHostName);

        if (hasTarget){
            tx = LimelightHelpers.getTX(LimelightConstants.limelightFloorHostName);
            desiredHeading = driveSubsystem.getHeading() + tx;
            desiredHeading = desiredHeading / 180 * Math.PI;
            targetFound = true;
            SmartDashboard.putNumber("tx intake", tx);
            SmartDashboard.putNumber("desiredHeading intake", desiredHeading);

        }

        if (targetFound) {
            // potentially get driver x and y input
            double xSpeed = -MathUtil.applyDeadband(xboxController.getLeftY(), OIConstants.kDriveDeadband);
            double rotSpeed = -driveSubsystem.getDesiredHeadingSpeed(desiredHeading);
            SmartDashboard.putNumber("xSpeed", xSpeed);
            SmartDashboard.putNumber("rotSpeed", rotSpeed);
            driveSubsystem.drive(xSpeed, 0, rotSpeed, false, true, true);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true, true, false);
        intakeSubsystem.runIntake(0);
        rollersSubsystem.index(0);
        rollersSubsystem.moveRoller(0);
        armSubsystem.resetIntegral();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // command only ends when interrupted
        SmartDashboard.putBoolean("switch", noteLimitSwitch.get());
        if (noteLimitSwitch.get()){
            return true;
        }
        return false;
    }
}