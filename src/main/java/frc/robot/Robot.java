// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AprilTagHeights;
import frc.robot.Constants.AprilTagIds;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.AprilTagIds.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* CameraServer.startAutomaticCapture(); */
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("hold position", m_robotContainer.m_ArmSubsystem.getHoldPosition());
    SmartDashboard.putNumber("encoder", m_robotContainer.m_ArmSubsystem.getEncoderPosition());

    double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight-april");
    double limelightMountAngleDegrees = 13.0;
    double limelightLensHeightInches = 5.875;
    double goalHeightInches;
    double id = LimelightHelpers.getFiducialID("limelight-april");
    if (id == AprilTagIds.stageBlueOne || id == AprilTagIds.stageBlueTwo || id == AprilTagIds.stageBlueThree) {
      goalHeightInches = AprilTagHeights.stage;
    } else if (id == AprilTagIds.speakerBlueLeft || id == AprilTagIds.speakerBlueRight
        || id == AprilTagIds.speakerRedLeft || id == AprilTagIds.speakerRedRight) {
      goalHeightInches = AprilTagHeights.speaker;
    } else if (id == AprilTagIds.sourceBlueRight || id == AprilTagIds.sourceBlueLeft || id == AprilTagIds.sourceRedLeft
        || id == AprilTagIds.sourceRedRight) {
      goalHeightInches = AprilTagHeights.source;
    } else if (id == AprilTagIds.ampBlue || id == AprilTagIds.ampRed) {
      goalHeightInches = AprilTagHeights.amp;
    } else {
      goalHeightInches = 0;
    }
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    double distanceToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    SmartDashboard.putNumber("Distance to wall", distanceToGoalInches);

    double distanceToTagInches = Math
        .sqrt(Math.pow(goalHeightInches - limelightLensHeightInches, 2) + Math.pow(distanceToGoalInches, 2));
    SmartDashboard.putNumber("Distance to tag", distanceToTagInches);
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
