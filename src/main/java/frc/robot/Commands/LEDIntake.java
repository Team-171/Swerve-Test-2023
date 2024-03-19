package frc.robot.Commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LedSubsystem;

public class LEDIntake extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    
    private LedSubsystem ledSubsystem;
    private DigitalInput noteLimitSwitch;

    /**
     * Follows a given trajectory for autonomous.
     * @param trajectory Trajectory to follow
     * @param subsystem Drive subsystem to drive the robot
     */
    public LEDIntake(LedSubsystem ledSubsystem, DigitalInput noteLimitSwitch) {
        this.ledSubsystem = ledSubsystem;
        this.noteLimitSwitch = noteLimitSwitch;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (noteLimitSwitch.get()){
            ledSubsystem.changeColor(LEDConstants.green);
        }
        else {
            ledSubsystem.changeColor(LEDConstants.red);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ledSubsystem.changeColor(LEDConstants.rainbowWithGlitter);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}