// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  // The subsystem's motor
  private final CANSparkMax canSparkMax;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    canSparkMax = new CANSparkMax(
      Constants.Intake.kPort, 
      MotorType.kBrushless);
    canSparkMax.setInverted(false);
    canSparkMax.set(0);
  }

  @Override
  public void periodic() {
    // if (this.canSparkMax.get() == -Constants.Intake.kSpeed) {
    //   SmartDashboard.putString("Intake State", "IN");
    // } else if (this.canSparkMax.get() == Constants.Intake.kSpeed) {
    //   SmartDashboard.putString("Intake State", "OUT");
    // }
    // else{
    //   SmartDashboard.putString("Intake State", "STOPPED");
    // }
  }

  public void SpinForwards() {
    canSparkMax.set(-Constants.Intake.kSpeed);
  }

  public void SpinBackwards() {
    canSparkMax.set(Constants.Intake.kSpeed);
  }

  public void Off() {
    canSparkMax.set(0);
  }
}
