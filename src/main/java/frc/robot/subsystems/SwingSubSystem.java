// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwingSubSystem extends SubsystemBase {

  private DoubleSolenoid doubleSolenoid;
  public SwingSubSystem() {

    doubleSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 3, 4);
    doubleSolenoid.set(Value.kReverse);
  }

public void Change(){
  Value value = doubleSolenoid.get() == Value.kForward ? Value.kReverse : Value.kForward;
  doubleSolenoid.set(value);
}

public void Up(){
  doubleSolenoid.set(Value.kForward);
}

public void Down(){
  doubleSolenoid.set(Value.kReverse);
}

  @Override
  public void periodic() {
    // if (this.doubleSolenoid.get() == Value.kForward) {
    //   SmartDashboard.putString("Swing State", "UP");
    // } else if (this.doubleSolenoid.get() == Value.kReverse) {
    //   SmartDashboard.putString("Swing State", "DOWN");
    // }
    // else{
    //   SmartDashboard.putString("Singulator State", "STOPPED");
    // }
  }
}
