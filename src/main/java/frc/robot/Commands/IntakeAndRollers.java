package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AprilTagHeights;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollersSubsystem;

public class IntakeAndRollers extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private RollersSubsystem rollersSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ArmSubsystem armSubsystem;
    private double intakeSpeed;
    private double rollerSpeed;
    private double indexSpeed;
    private double armPosition;
    private double shootPosition;
    private double targetDistance;

    /**
     * Follows a given trajectory for autonomous.
     * 
     * @param trajectory Trajectory to follow
     * @param subsystem  Drive subsystem to drive the robot
     */
    public IntakeAndRollers(RollersSubsystem rollersSubsystem, IntakeSubsystem intakeSubsystem,
            ArmSubsystem armSubsystem, double rollerSpeed, double indexSpeed, double intakeSpeed, double armPosition) {
        this.rollersSubsystem = rollersSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
        this.rollerSpeed = rollerSpeed;
        this.indexSpeed = indexSpeed;
        this.intakeSpeed = intakeSpeed;
        this.armPosition = armPosition;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(rollersSubsystem, intakeSubsystem, armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double ty = LimelightHelpers.getTY(LimelightConstants.limelightAprilHostName);

        targetDistance = (AprilTagHeights.speaker - LimelightConstants.speakerLimelightHeight)
                / Math.tan((LimelightConstants.speakerLimelightAngle + ty) * Math.PI / 180);

        double td = targetDistance / 12;

        SmartDashboard.putNumber("td IntakeAndRollers", td);

        shootPosition = (-0.000001 * Math.pow(td, 4)) + (0.00006 * Math.pow(td, 3))
                - (0.0015 * Math.pow(td, 2)) + (0.0209 * td) + 0.6365;

        armSubsystem.setPointArm(armPosition);
        intakeSubsystem.runIntake(intakeSpeed);
        rollersSubsystem.index(indexSpeed);
        rollersSubsystem.moveRoller(rollerSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}