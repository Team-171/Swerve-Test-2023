package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollersSubsystem;

public class AutoIntake extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private RollersSubsystem rollersSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private IndexSubsystem indexSubsystem;
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
    private DigitalInput noteLimitSwitch;

    /**
     * Follows a given trajectory for autonomous.
     * 
     * @param trajectory Trajectory to follow
     * @param subsystem  Drive subsystem to drive the robot
     */
    public AutoIntake(RollersSubsystem rollersSubsystem, IntakeSubsystem intakeSubsystem,
            IndexSubsystem indexSubsystem,
            ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem,
            DigitalInput noteLimitSwitch) {

        this.rollersSubsystem = rollersSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.indexSubsystem = indexSubsystem;
        this.rollerSpeed = RollerConstants.intakeRollerSpeed;
        this.indexSpeed = IndexConstants.intakeIndexSpeed;
        this.intakeSpeed = IntakeConstants.intakeSpeed;
        this.armPosition = ArmConstants.lowStop;
        this.noteLimitSwitch = noteLimitSwitch;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(rollersSubsystem, intakeSubsystem, indexSubsystem, armSubsystem, driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        armSubsystem.setPointArm(armPosition);
        hasTarget = false;
        targetFound = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (armSubsystem.getEncoderPosition() > armPosition - 0.05) {
            intakeSubsystem.runIntake(intakeSpeed);
            indexSubsystem.moveIndex(indexSpeed);
            rollersSubsystem.moveRoller(rollerSpeed);
            armSubsystem.resetIntegral();
        }

        hasTarget = LimelightHelpers.getTV(LimelightConstants.limelightFloorHostName);

        if (hasTarget) {
            tx = LimelightHelpers.getTX(LimelightConstants.limelightFloorHostName);
            desiredHeading = driveSubsystem.getHeading() + tx;
            desiredHeading = desiredHeading / 180 * Math.PI;
            targetFound = true;
        }

        if (targetFound) {
            // potentially get driver x and y input
            double xSpeed;
            double ySpeed;
            xSpeed = AutoConstants.intakeDriveSpeed;
            ySpeed = 0;
            double rotSpeed = -driveSubsystem.getDesiredHeadingSpeed(desiredHeading);
            driveSubsystem.drive(xSpeed, ySpeed, rotSpeed, false, true, true);
        }
        armSubsystem.setPointArm(armPosition);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true, true, false);
        intakeSubsystem.runIntake(0);
        indexSubsystem.moveIndex(0);
        rollersSubsystem.moveRoller(0);
        armSubsystem.resetIntegral();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (noteLimitSwitch.get()) {
            return true;
        }
        return false;
    }
}