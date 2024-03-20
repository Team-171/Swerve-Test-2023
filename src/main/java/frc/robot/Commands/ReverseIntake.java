package frc.robot.Commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollersSubsystem;

public class ReverseIntake extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private RollersSubsystem rollersSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private IndexSubsystem indexSubsystem;
    private double intakeSpeed;
    private double rollerSpeed;
    private double indexSpeed;
    private DigitalInput noteSwitch;

    /**
     * Follows a given trajectory for autonomous.
     * 
     * @param trajectory Trajectory to follow
     * @param subsystem  Drive subsystem to drive the robot
     */
    public ReverseIntake(RollersSubsystem rollersSubsystem, IntakeSubsystem intakeSubsystem,
            IndexSubsystem indexSubsystem, DigitalInput noteSwitch) {
        this.rollersSubsystem = rollersSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.indexSubsystem = indexSubsystem;
        this.rollerSpeed = RollerConstants.intakeRollerSpeed;
        this.indexSpeed = IndexConstants.intakeIndexSpeed;
        this.intakeSpeed = IntakeConstants.intakeSpeed;
        this.noteSwitch = noteSwitch;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(rollersSubsystem, intakeSubsystem, indexSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (noteSwitch.get()) {
            intakeSubsystem.runIntake(-intakeSpeed);
        }else{
            intakeSubsystem.runIntake(-intakeSpeed);
            indexSubsystem.moveIndex(-indexSpeed);
            rollersSubsystem.moveRoller(-rollerSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.runIntake(0);
        indexSubsystem.moveIndex(0);
        rollersSubsystem.moveRoller(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // command only ends when interrupted
        return false;
    }
}