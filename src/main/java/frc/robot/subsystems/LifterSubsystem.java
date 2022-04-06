// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LifterSubsystem extends SubsystemBase {
  // The subsystem's motor
  private final CANSparkMax canSparkMax;

  /** Creates a new LifterSubsystem. */
  public LifterSubsystem() {
    canSparkMax = new CANSparkMax(
        Constants.Lifter.kPort,
        MotorType.kBrushless);
    canSparkMax.setInverted(false);
    canSparkMax.set(0);
    canSparkMax.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // if (this.canSparkMax.get() == Constants.Lifter.kSpeed) {
    //   SmartDashboard.putString("Lifter State", "LIFTING");
    // } else if (this.canSparkMax.get() == -Constants.Lifter.kSpeed) {
    //   SmartDashboard.putString("Lifter State", "LOWERING");
    // }
    // else{
    //   SmartDashboard.putString("Lifter State", "STOPPED");
    // }
  }

  public void SpinForwards() {
    canSparkMax.set(Constants.Lifter.kSpeed);
  }

  public void SpinBackwards() {
    canSparkMax.set(-Constants.Lifter.kSpeed);
  }

  public void Off() {
    canSparkMax.set(0);
  }
}
