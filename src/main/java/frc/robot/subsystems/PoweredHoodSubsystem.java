// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoweredHoodSubsystem extends SubsystemBase {

  private final CANSparkMax canSparkMax;

  public PoweredHoodSubsystem() {
    this.canSparkMax = new CANSparkMax(Constants.PoweredHood.kPort, MotorType.kBrushless);
    this.canSparkMax.setInverted(false);
    this.canSparkMax.set(0);
    canSparkMax.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SpinForwards(){
    canSparkMax.set(Constants.PoweredHood.speed);
  }

  public void SpinBackwards(){
    canSparkMax.set(-Constants.PoweredHood.speed);
  }

  public void Off(){
    canSparkMax.set(0);
  }

}
