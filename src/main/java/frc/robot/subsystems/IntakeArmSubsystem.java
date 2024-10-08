// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArmSubsystem extends SubsystemBase {
  private final Solenoid solenoid;
  private boolean intakeArmState;

  public IntakeArmSubsystem() {
    solenoid = new Solenoid(
        2, PneumaticsModuleType.REVPH, Constants.IntakeArm.kForwardChannel);

    intakeArmState = false;
    solenoid.set(intakeArmState);
  }

  @Override
  public void periodic() {
    // if (this.intakeArmState) {
    //   SmartDashboard.putString("Intake Arm State", "DOWN");
    // }
    // else{
    //   SmartDashboard.putString("Intake Arm State", "UP");
    // }
  }

  public void Change() {
    intakeArmState = !intakeArmState;
    solenoid.set(intakeArmState);
  }

  public void ArmUp() {
    intakeArmState = false;
    solenoid.set(intakeArmState);
  }

  public void ArmDown() {
    intakeArmState = true;
    solenoid.set(intakeArmState);
  }
}
