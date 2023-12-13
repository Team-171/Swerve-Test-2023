package frc.robot.Commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowPath extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final PathPlannerTrajectory path;
    private final DriveSubsystem driveSubsystem;
    private PathPlannerState state;
    private Pose2d currentPose;
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private ProfiledPIDController rotPid;
    private HolonomicDriveController hController;

    private final Timer timer = new Timer();

    /**
     * Follows a given trajectory for autonomous.
     * @param trajectory Trajectory to follow
     * @param subsystem Drive subsystem to drive the robot
     */
    public FollowPath(PathPlannerTrajectory trajectory, DriveSubsystem subsystem) {
        path = trajectory;
        driveSubsystem = subsystem;
        rotPid = Constants.AutoConstants.kRotController;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rotPid.enableContinuousInput(-Math.PI, Math.PI);
        hController = new HolonomicDriveController(Constants.AutoConstants.kXController,
                Constants.AutoConstants.kYController, rotPid);

        timer.stop();
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var currentTime = timer.get();
        state = (PathPlannerState) path.sample(currentTime);
        currentPose = driveSubsystem.getPose();
        speeds = hController.calculate(currentPose, state, state.holonomicRotation);
        driveSubsystem.updateStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}