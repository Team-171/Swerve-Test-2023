package frc.robot.Commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;

public class Rumble extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    
    private XboxController xboxController;
    private DigitalInput noteLimitSwitch;

    /**
     * Follows a given trajectory for autonomous.
     * @param trajectory Trajectory to follow
     * @param subsystem Drive subsystem to drive the robot
     */
    public Rumble(XboxController xboxController, DigitalInput noteLimitSwitch) {
        this.xboxController = xboxController;
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
            xboxController.setRumble(RumbleType.kBothRumble, 0.75);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        xboxController.setRumble(RumbleType.kBothRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return false;
    }
}