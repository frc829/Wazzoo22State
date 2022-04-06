// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PincerSubsystem extends SubsystemBase {
  private final Solenoid solenoid;
  private boolean pincerState;

  public PincerSubsystem() {
    solenoid = new Solenoid(
        2, PneumaticsModuleType.REVPH,
        Constants.Pincer.kForwardChannel);
    pincerState = false;
    solenoid.set(pincerState);

  }

  @Override
  public void periodic() {
    // if (this.pincerState) {
    //   SmartDashboard.putString("Pincer State", "OPEN");
    //   SmartDashboard.putString("Shooter Aim State", "HIGH");
    // }
    // else{
    //   SmartDashboard.putString("Pincer State", "CLOSED");
    //   SmartDashboard.putString("Shooter Aim State", "LOW or FAR");
    // }
  }

  public void Change() {
    pincerState = !pincerState;
    solenoid.set(pincerState);
  }

  public void PincerOpen() {
    pincerState = false;
    solenoid.set(pincerState);
  }

  public void PincerClosed() {
    pincerState = true;
    solenoid.set(pincerState);
  }
}
