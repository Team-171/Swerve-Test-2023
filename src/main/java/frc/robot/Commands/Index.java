package frc.robot.Commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.RollersSubsystem;

public class Index extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    
    private RollersSubsystem subsystem;
    private double speed;
    private DigitalInput noteLimitSwitch;

    /**
     * Follows a given trajectory for autonomous.
     * @param trajectory Trajectory to follow
     * @param subsystem Drive subsystem to drive the robot
     */
    public Index(RollersSubsystem subsystem, DigitalInput noteLimitSwitch) {
        this.subsystem = subsystem;
        this.speed = -1;
        this.noteLimitSwitch = noteLimitSwitch;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        subsystem.index(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.index(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!noteLimitSwitch.get()) {
            return true;
        }
        return false;
    }
}