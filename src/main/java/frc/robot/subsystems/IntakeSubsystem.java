// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
  // Initialization of variables
  CANSparkMax intakeMotor;
  PIDController pid;
  double setDistance;
  double holdPosition;

  /**
   * Creates a new IntakeRollersSubsystem.
   * Controls speed of intake rollers
   */
  public IntakeSubsystem() {
    // Creates a roller for intake
    intakeMotor = new CANSparkMax(15, MotorType.kBrushless);

    // Resets to default, always do before changing config
    intakeMotor.restoreFactoryDefaults();

    intakeMotor.setClosedLoopRampRate(0.125);
  }

  /**
   * Moves the roller based on an axis, in this case, triggers
   * 
   * @param speed double Input from triggers / axis
   */
  public void runIntake(double speed) {

    // Set speed and reset encoder
    // If not moving, try to get back to what it was when it stopped moving
    intakeMotor.set(speed);
  }

  /**
   * Resets the encoder in the motor
   */
  public void reset() {
    intakeMotor.getEncoder().setPosition(0);
  }

  public void index(double speed) {
    intakeMotor.set(speed);
  }

  //public void rumbleController(Rev2mDistanceSensor sensor, XboxController controller){
  //  controller.setRumble(RumbleType.kBothRumble, 1);
  //}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
