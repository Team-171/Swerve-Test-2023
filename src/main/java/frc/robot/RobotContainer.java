// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.AimAndRev;
import frc.robot.Commands.Index;
import frc.robot.Commands.IntakeAndRollers;
import frc.robot.Commands.ZeroHeading;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollersSubsystem;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final RollersSubsystem m_RollersSubsystem = new RollersSubsystem();
        public final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
        private final LedSubsystem m_ChangeLedSubsystem = new LedSubsystem();
        private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
        private boolean slowmode;
        //private final PIDController driveTurnPID = new PIDController(DriveConstants.kDriveTurnP, DriveConstants.kDriveTurnI, DriveConstants.kDriveTurnD);
        //static double theta = 0;
        

        private final SendableChooser<Command> autoChooser;
        public final SendableChooser<Double> ledChooser = new SendableChooser<Double>();

        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                autoChooser = AutoBuilder.buildAutoChooser("Straight");
                SmartDashboard.putData("Auto Chooser", autoChooser);
                slowmode = false;

                // NamedCommands.registerCommand("Index", new Index(m_RollersSubsystem, 0));

                ledChooser.setDefaultOption("Rainbow", Constants.LEDConstants.rainbowWithGlitter);
                ledChooser.addOption("Twinkles", Constants.LEDConstants.twinkleColor1Color2);
                ledChooser.addOption("Cheese", Constants.LEDConstants.cheese);
                ledChooser.addOption("Heartbeat", Constants.LEDConstants.heartbeatFastColor1);
                ledChooser.addOption("Color Gradient", Constants.LEDConstants.colorGradient);
                ledChooser.addOption("Larson Scanner", Constants.LEDConstants.larsonScanner);
                ledChooser.addOption("Strobe", Constants.LEDConstants.fixedStrobeGold);
                SmartDashboard.putData(ledChooser);

                //theta = m_robotDrive.getDegrees();

                // m_robotDrive.resetEncoders();

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true, true, slowmode),
                                                m_robotDrive));

                /*m_robotDrive.setDefaultCommand(
                                new RunCommand(() -> 
                                { 
                                        double leftY = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
                                        double leftX = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
                                        double rightX = -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);
                                        double turnInput = 0;
                                        // if recieving joystick input drive normally
                                        if (rightX != 0) {
                                                theta = m_robotDrive.getDegrees();
                                                turnInput = rightX;
                                        } 
                                        // else maintain heading
                                        else {
                                                turnInput = driveTurnPID.calculate(m_robotDrive.getDegrees(), theta);
                                        }
                                        m_robotDrive.drive(leftY, leftX, turnInput, true, true, slowmode);
                                }, m_robotDrive));*/

                m_ArmSubsystem.setDefaultCommand(
                                new RunCommand(() -> m_ArmSubsystem.moveArm(
                                                -MathUtil.applyDeadband(m_driverController.getRightTriggerAxis(),
                                                                OIConstants.kDriveDeadband),
                                                -MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(),
                                                                OIConstants.kDriveDeadband)),
                                                m_ArmSubsystem));

                m_ChangeLedSubsystem.setDefaultCommand(
                                new RunCommand(() -> m_ChangeLedSubsystem.changeColor(ledChooser.getSelected()),
                                                m_ChangeLedSubsystem));

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.setX(),
                                                m_robotDrive));

                new JoystickButton(m_driverController, ButtonConstants.ButtonSelect) // select button
                                .onTrue(new ZeroHeading(m_robotDrive));

                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(new AimAndRev(m_robotDrive));



                // intake a note from ground
                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .whileTrue(new IntakeAndRollers(m_RollersSubsystem, m_IntakeSubsystem, m_ArmSubsystem, RollerConstants.intakeRollerSpeed, IndexConstants.intakeIndexSpeed, IntakeConstants.intakeSpeed, ArmConstants.lowStop))
                                .whileFalse(new IntakeAndRollers(m_RollersSubsystem, m_IntakeSubsystem,  m_ArmSubsystem, 0, 0, 0, ArmConstants.lowStop));
                
                
                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .whileTrue(new IntakeAndRollers(m_RollersSubsystem, m_IntakeSubsystem, m_ArmSubsystem, RollerConstants.outputRollerSpeed, 0, 0, ArmConstants.speakerPos))
                                .whileFalse(new IntakeAndRollers(m_RollersSubsystem, m_IntakeSubsystem, m_ArmSubsystem, 0, 0, 0, ArmConstants.speakerPos));

                // keep in
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(new Index(m_RollersSubsystem, 1))
                                .whileFalse(new Index(m_RollersSubsystem, 0));
                
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(new RunCommand(() -> slowmode = true))
                                .whileFalse(new RunCommand(() -> slowmode = false));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                Field2d m_Field2d = new Field2d();
                SmartDashboard.putData(m_Field2d);

                /*
                 * List<PathPlannerTrajectory> autoPaths1 =
                 * PathPlanner.loadPathGroup(autoChooser.getSelected(),
                 * Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                 * Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
                 * 
                 * Command autoTest = new SequentialCommandGroup(
                 * new FollowPathWithEvents(
                 * new FollowPath(autoPaths1.get(0), m_robotDrive),
                 * autoPaths1.get(0).getMarkers(),
                 * Constants.AutoConstants.AUTO_EVENT_MAP)
                 * );
                 * 
                 * 
                 * m_Field2d.getObject("traj").setTrajectory(autoPaths1.get(0));
                 * 
                 * return autoTest;
                 */

                return autoChooser.getSelected();
        }
}
