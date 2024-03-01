package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RollersSubsystem;

public class CorrectTurning extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    
    private DriveSubsystem subsystem;
    private static double theta = 0;
    private double xSpeedController;
    private double ySpeedController;
    private double thetaSpeedController;
    private final PIDController driveTurnPID = new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);


    /**
     * Follows a given trajectory for autonomous.
     * @param trajectory Trajectory to follow
     * @param subsystem Drive subsystem to drive the robot
     */
    public CorrectTurning(DriveSubsystem subsystem, double xSpeedController, double ySpeedController, double thetaSpeedController) {
        this.subsystem = subsystem;
        this.xSpeedController = xSpeedController;
        this.ySpeedController = ySpeedController;
        this.thetaSpeedController = thetaSpeedController;

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
        double turnInput = 0;
        // if recieving joystick input drive normally
        if (thetaSpeedController != 0){
            theta = subsystem.getDegrees();
            turnInput = thetaSpeedController;
        } 
        // else maintain heading
        else{
            //turnInput = driveTurnPID.calculate(subsystem.getDegrees(), theta);
        }
        subsystem.drive(xSpeedController, ySpeedController, turnInput, true, true);
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