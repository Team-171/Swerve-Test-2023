// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;


public class ArmSubsystem extends SubsystemBase {
  // Initialization of variables
  CANSparkMax armMotor;
  CANSparkMax armMotor2;
  PIDController pid;
  double setDistance;
  double holdPosition;
  DutyCycleEncoder encoder;
  double setPower;
  double setPoint;
  boolean firstHoldPositionSet = false;

  /** 
   * Creates a new IntakeRollersSubsystem.
   * Controls speed of intake rollers 
  */
  public ArmSubsystem() {
    // Creates a roller for intake
    armMotor = new CANSparkMax(13, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    encoder = new DutyCycleEncoder(9);

    pid = new PIDController(10, 0, 0);

    setPower = 0;

    // Resets to default, always do before changing config
    armMotor.restoreFactoryDefaults();
    armMotor2.restoreFactoryDefaults();

    armMotor.setSmartCurrentLimit(60);
    armMotor2.setSmartCurrentLimit(60);

    armMotor.setClosedLoopRampRate(0.1);
    armMotor2.setClosedLoopRampRate(0.1);

    armMotor2.setInverted(true);
  }

  /**
   * Moves the roller based on an axis, in this case, triggers
   * @param speed double Input from triggers / axis
   */
  public void moveArm(double speedRight, double speedLeft){
    SmartDashboard.putNumber("tyApril", LimelightHelpers.getTY(LimelightConstants.limelightAprilHostName));
    SmartDashboard.putNumber("tyFloor", LimelightHelpers.getTY(LimelightConstants.limelightFloorHostName));
    if (!firstHoldPositionSet){
      holdPosition = encoder.getAbsolutePosition();
      firstHoldPositionSet = true;
    }
    double speed = speedRight - speedLeft;
    speed = speed * .05;

    setPoint = MathUtil.clamp(speed + holdPosition, ArmConstants.highStop, ArmConstants.lowStop);

    setPower = MathUtil.clamp(pid.calculate(encoder.getAbsolutePosition(), setPoint), -ArmConstants.speed, ArmConstants.speed);
    
    if (speed != 0){
        holdPosition = encoder.getAbsolutePosition();
    }
    armMotor.set(setPower);
    armMotor2.set(setPower);
  }

  public boolean setPointArm(double position){
    setPoint = position;
    holdPosition = position;

    setPower = MathUtil.clamp(pid.calculate(encoder.getAbsolutePosition(), setPoint), -ArmConstants.speed, ArmConstants.speed);

    armMotor.set(setPower);
    armMotor2.set(setPower);

    return false;
  }

  public double getHoldPosition(){
    return holdPosition;
  }

  public double getEncoderPosition(){
    return encoder.getAbsolutePosition();
  }

  public void resetIntegral(){
    pid.reset();
  }

  /**
   * Resets the encoder in the motor
   */
  public void reset(){
    armMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("hold position", getHoldPosition());
    SmartDashboard.putNumber("encoder", getEncoderPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
