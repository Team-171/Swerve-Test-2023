// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  /** 
   * Creates a new IntakeRollersSubsystem.
   * Controls speed of intake rollers 
  */
  public ArmSubsystem() {
    // Creates a roller for intake
    armMotor = new CANSparkMax(13, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    encoder = new DutyCycleEncoder(9);

    pid = new PIDController(0.5, 0, 0);

    setPower = 0;

    holdPosition = 0.708;

    // Resets to default, always do before changing config
    armMotor.restoreFactoryDefaults();
    armMotor2.restoreFactoryDefaults();

    armMotor.setClosedLoopRampRate(0.125);
    armMotor2.setClosedLoopRampRate(0.125);

    armMotor2.setInverted(true);
  }

  /**
   * Moves the roller based on an axis, in this case, triggers
   * @param speed double Input from triggers / axis
   */
  public void moveArm(double speedRight, double speedLeft){
    double speed = speedRight - speedLeft;
    speed = speed * 0.5;

    /* setPoint = MathUtil.clamp(speed + holdPosition, ArmConstants.lowStop, ArmConstants.highStop);

    setPower = MathUtil.clamp(pid.calculate(encoder.getAbsolutePosition(), setPoint), -ArmConstants.speed, ArmConstants.speed);
    
    if (speed != 0){
        holdPosition = encoder.getAbsolutePosition();
    }

    armMotor.set(setPower);
    armMotor2.set(setPower); */ 

    armMotor.set(speed);
    armMotor2.set(speed);
  }

  public boolean setPointArm(double position){
    setPoint = position;
    holdPosition = position;

    setPower = -MathUtil.clamp(pid.calculate(encoder.getAbsolutePosition(), setPoint), -ArmConstants.speed, ArmConstants.speed);

    armMotor.set(setPower);
    armMotor2.set(setPower);

    return false;

    //Speaker - 0.35
    //down - 0.56
  }

  /**
   * Resets the encoder in the motor
   */
  public void reset(){
    armMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder", encoder.getAbsolutePosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
