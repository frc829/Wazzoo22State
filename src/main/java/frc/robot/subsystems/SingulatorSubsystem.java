// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SingulatorSubsystem extends SubsystemBase {
  // The subsystem's motor
  private final CANSparkMax canSparkMax;

  /** Creates a new SingulatorSubsystem. */
  public SingulatorSubsystem() {
    canSparkMax = new CANSparkMax(
        Constants.Singulator.kPort,
        MotorType.kBrushless);
    canSparkMax.setInverted(true);
    canSparkMax.setIdleMode(IdleMode.kBrake);
    
    
    canSparkMax.set(0);
  }

  @Override
  public void periodic() {
    // if (this.canSparkMax.get() == Constants.Lifter.kSpeed) {
    //   SmartDashboard.putString("Singulator State", "FIRING");
    // } else if (this.canSparkMax.get() == -Constants.Lifter.kSpeed) {
    //   SmartDashboard.putString("Singulator State", "HOLDING");
    // }
    // else{
    //   SmartDashboard.putString("Singulator State", "STOPPED");
    // }

  }

  public void SpinForward() {
    canSparkMax.set(Constants.Singulator.kSpeed);
  }

  public void SpinBackwards() {
    canSparkMax.set(-Constants.Singulator.kSpeed);
  }

  public void Off() {
    canSparkMax.set(0);
  }
}
