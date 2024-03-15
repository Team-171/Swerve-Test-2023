// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IndexSubsystem extends SubsystemBase {
  // Initialization of variables
  CANSparkMax indexer;

  /**
   * Creates a new IndexSubsystem.
   * Controls speed of index rollers
   */
  public IndexSubsystem() {
    // Creates a roller for intake
    indexer = new CANSparkMax(11, MotorType.kBrushless);

    // Resets to default, always do before changing config
    indexer.restoreFactoryDefaults();

    indexer.setClosedLoopRampRate(0.125);
  }

  /**
   * Resets the encoder in the motor
   */
  public void reset() {
    indexer.getEncoder().setPosition(0);
  }

  public void moveIndex(double speed) {
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
