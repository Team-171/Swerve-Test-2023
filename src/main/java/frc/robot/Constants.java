// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 2.2; // radians per second
    public static final double kMagnitudeSlewRate = 2.4; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(20.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.5);
    // Distance between front and back wheels on robot
    
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 9;

    public static final boolean kGyroReversed = true;

    public static final double kDriveTurnP = 0.04;
    public static final double kDriveTurnI = 0;
    public static final double kDriveTurnD = 0;

    public static final double kHeadingP = 0.6;
    public static final double kHeadingI = 0;
    public static final double kHeadingD = 0;

    public static final double rotSpeedCoefficient = 0.8;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.01;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 0.8;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0.0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class ButtonConstants {
    public static final int RightBumper = 6;
    public static final int LeftBumper = 5;
    public static final int ButtonB = 2;
    public static final int ButtonSelect = 7;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.15;
  }

  public static final class LimelightConstants {
    public static final double kUpperTolerance = 2.0;
    public static final double kLowerTolerance = -2.0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double speakerLimelightHeight = 7.75;
    public static final double speakerLimelightAngle = 23;
    public static final double xOffset = 0; // right is positive
    public static final double yOffset = 0;
    public static final String limelightAprilHostName = "limelight-allison";
    public static final String limelightFloorHostName = "limelight-floor";
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

    public static final TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAccelerationMetersPerSecondSquared);

    public static final PIDController kXController = new PIDController(.23, 0, 0);
    public static final PIDController kYController = new PIDController(.00717, 0, 0);
    public static final ProfiledPIDController kRotController = new ProfiledPIDController(0.415, 0, 0, rotProfile);

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double intakeDriveSpeed = 0.5;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class AprilTagIds {
    public static final double sourceBlueRight = 1;
    public static final double sourceBlueLeft = 2;
    public static final double speakerRedRight = 3;
    public static final double speakerRedLeft = 4;
    public static final double ampRed = 5;
    public static final double ampBlue = 6;
    public static final double speakerBlueRight = 7;
    public static final double speakerBlueLeft = 8;
    public static final double sourceRedRight = 9;
    public static final double sourceRedLeft = 10;
    public static final double stageRedOne = 11;
    public static final double stageRedTwo = 12;
    public static final double stageRedThree = 13;
    public static final double stageBlueOne = 14;
    public static final double stageBlueTwo = 15;
    public static final double stageBlueThree = 16;
  }

  public static final class AprilTagHeights {
    public static final double source = 48.126;
    public static final double amp = 50.13;
    public static final double speaker = 53.875;
    public static final double stage = 48.841;
  }

  public static final class ArmConstants {
    public static final double lowStop = 0.903;
    public static final double highStop = 0.55;
    public static final double speed = 0.75;
    public static final double speakerPos = 0.69;
    public static final double ampPos = 0.575;
  }

  public static final class RollerConstants {
    public static final double intakeRollerSpeed = -0.4;
    public static final double outputRollerSpeed = 0.75;
  }

  
  public static final class IndexConstants {
    public static final double intakeIndexSpeed = 0.3;
  }

  public static final class IntakeConstants {
    public static final double intakeSpeed = -1;
  }

  public static final class LEDConstants {
    public static final double rainbowWithGlitter = -0.89;
    public static final double heartbeatFastColor1 = 0.27;
    public static final double cheese = 0.67;
    public static final double twinkleColor1Color2 = 0.49;
    public static final double colorGradient = 0.41;
    public static final double larsonScanner = -0.33;
    public static final double fixedStrobeGold = -0.07;
    public static final double green = 0.77;
    public static final double red = 0.61;
    public static final double violet = 0.91;
    public static final double redBreathe = -0.17;
    public static final double blueBreathe = -0.15;
    public static final double redHeartbeat = -0.25;
    public static final double blueHeartbeat = -0.23;
    public static final int channel = 9;
  }

  public static final class LimitSwitchConstants {
    public static final int channel = 8;
  }

}
