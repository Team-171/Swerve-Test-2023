// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;

public class RollersSubsystem extends SubsystemBase {
  // Initialization of variables
  CANSparkMax rollersMotor;
  CANSparkMax rollersMotor2;
  PIDController pid;
  double setDistance;
  double holdPosition;
  CANSparkMax indexer;

  /**
   * Creates a new IntakeRollersSubsystem.
   * Controls speed of intake rollers
   */
  public RollersSubsystem() {
    // Creates a roller for intake
    rollersMotor = new CANSparkMax(12, MotorType.kBrushless);
    rollersMotor2 = new CANSparkMax(10, MotorType.kBrushless);
    indexer = new CANSparkMax(11, MotorType.kBrushless);

    // Resets to default, always do before changing config
    indexer.restoreFactoryDefaults();
    rollersMotor.restoreFactoryDefaults();
    rollersMotor2.restoreFactoryDefaults();

    rollersMotor.setClosedLoopRampRate(0.5);
    rollersMotor2.setClosedLoopRampRate(0.5);
    indexer.setClosedLoopRampRate(0.125);

  }

  /**
   * Moves the roller based on an axis, in this case, triggers
   * 
   * @param speed double Input from triggers / axis
   */
  public void moveRoller(double speed) {

    // Set speed and reset encoder
    // If not moving, try to get back to what it was when it stopped moving
    rollersMotor.set(speed);
    rollersMotor2.set(-speed);
  }

  /**
   * Resets the encoder in the motor
   */
  public void reset() {
    rollersMotor.getEncoder().setPosition(0);
  }

  public void index(double speed) {
    indexer.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
