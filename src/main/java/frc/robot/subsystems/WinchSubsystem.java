// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WinchSubsystem extends SubsystemBase {

  private final CANSparkMax canSparkMax;
  private final DigitalInput encoderChannel;
  private final DutyCycleEncoder encoder;
  private final double encoderStartPosition;
  private double encoderPosition;
  private double armHeight;
  private double winchSpeed;

  /** Creates a new Winch. */
  public WinchSubsystem() {
    canSparkMax = new CANSparkMax(
        Constants.Winch.kPort, MotorType.kBrushless);
    canSparkMax.setIdleMode(IdleMode.kBrake);
    canSparkMax.setInverted(true);
    encoderChannel = new DigitalInput(8);
    encoder = new DutyCycleEncoder(encoderChannel);
    encoderStartPosition = encoder.getDistance();
    encoderPosition = encoder.getDistance();
    armHeight = Math.abs(encoderStartPosition - encoderPosition);
    winchSpeed = 0;
  }

  @Override
  public void periodic() {
    encoderPosition = encoder.getDistance();
    this.armHeight = Math.abs(encoderStartPosition - encoderPosition);
    // SmartDashboard.putNumber("WinchSpeed" , this.winchSpeed);
    // SmartDashboard.putNumber("WinchPosition", encoderPosition);
    
  }

  public void Winch(XboxController operatorControls) {
    DoubleSupplier leftYAxis = () -> operatorControls.getLeftY();
    this.winchSpeed = -leftYAxis.getAsDouble();
    if (Math.abs(winchSpeed) < 0.1) {
      winchSpeed = 0;
    }
    if (armHeight >= Constants.Winch.kMaxDistance && winchSpeed > 0) {
      canSparkMax.set(0);
    } else {
      canSparkMax.set(Constants.Winch.kSpeed * winchSpeed);
    }
  }
}
