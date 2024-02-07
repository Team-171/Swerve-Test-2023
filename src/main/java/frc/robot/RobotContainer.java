// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.AimAndShoot;
import frc.robot.Commands.ZeroHeading;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(null);

        private final SendableChooser<Command> autoChooser;

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

                // m_robotDrive.resetEncoders();

                // Configure default commands
                m_SwerveSubsystem.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                /*
                                 * new RunCommand(
                                 * () -> m_robotDrive.drive(
                                 * -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                 * OIConstants.kDriveDeadband),
                                 * -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                 * OIConstants.kDriveDeadband),
                                 * -MathUtil.applyDeadband(m_driverController.getRightX(),
                                 * OIConstants.kDriveDeadband),
                                 * true, true),
                                 * m_robotDrive));
                                 */
                                new RunCommand(
                                        () -> m_SwerveSubsystem.drive(
                                                        new Translation2d(-MathUtil.applyDeadband(
                                                                        m_driverController.getLeftY(),
                                                                        OIConstants.kDriveDeadband),
                                                                        -MathUtil.applyDeadband(
                                                                                        m_driverController
                                                                                                        .getLeftX(),
                                                                                        OIConstants.kDriveDeadband)),
                                                        -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                        OIConstants.kDriveDeadband),
                                                        true),
                                        m_SwerveSubsystem));
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
                new JoystickButton(m_driverController, Button.kR1.value)
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.setX(),
                                                m_robotDrive));

                new JoystickButton(m_driverController, ButtonConstants.ButtonSelect) // select button
                                .onTrue(new ZeroHeading(m_robotDrive));

                new JoystickButton(m_driverController, ButtonConstants.LeftBumper)
                                .whileTrue(new AimAndShoot(m_robotDrive));

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
