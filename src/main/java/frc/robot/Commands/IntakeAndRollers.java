package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollersSubsystem;

public class IntakeAndRollers extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    
    private RollersSubsystem rollersSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private double indexSpeed;
    private double rollerSpeed;
    private double intakeSpeed;

    /**
     * Follows a given trajectory for autonomous.
     * @param trajectory Trajectory to follow
     * @param subsystem Drive subsystem to drive the robot
     */
    public IntakeAndRollers(RollersSubsystem rollersSubsystem, IntakeSubsystem intakeSubsystem, double rollerSpeed, double indexSpeed, double intakeSpeed) {
        this.rollersSubsystem = rollersSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.indexSpeed = indexSpeed;
        this.rollerSpeed = rollerSpeed;
        this.intakeSpeed = intakeSpeed;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(rollersSubsystem, intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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