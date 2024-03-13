package frc.robot.Commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollersSubsystem;

public class Amp extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private RollersSubsystem rollersSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ArmSubsystem armSubsystem;
    private double intakeSpeed;
    private double rollerSpeed;
    private double indexSpeed;
    private double armPosition;
    private boolean running;
    private DigitalInput noteLimitSwitch;

    /**
     * Follows a given trajectory for autonomous.
     * 
     * @param trajectory Trajectory to follow
     * @param subsystem  Drive s
     * ubsystem to drive the robot
     */
    public Amp(RollersSubsystem rollersSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, DigitalInput noteLimitSwitch,
            boolean running) {
        this.rollersSubsystem = rollersSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
        this.rollerSpeed = -0.4;
        this.indexSpeed = 0.6;
        this.armPosition = ArmConstants.ampPos;
        this.running = running;
        this.noteLimitSwitch = noteLimitSwitch;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(rollersSubsystem, intakeSubsystem, armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        armSubsystem.setPointArm(armPosition);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (armSubsystem.getEncoderPosition() < armPosition + 0.05 && armSubsystem.getEncoderPosition() > armPosition - 0.05){
            intakeSubsystem.runIntake(intakeSpeed);
            rollersSubsystem.moveRoller(rollerSpeed);
            rollersSubsystem.index(indexSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.runIntake(0);
        rollersSubsystem.moveRoller(0);
        rollersSubsystem.index(0);
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