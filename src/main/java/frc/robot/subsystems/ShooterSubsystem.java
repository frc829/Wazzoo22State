// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;

public class ShooterSubsystem extends SubsystemBase {
  // The subsystem's motor
  private final WPI_TalonFX talonFXStarboard;
  private final WPI_TalonFX talonFXPort;

  private double shooterSpeed = 0;

  /**
   * Creates a new ShooterSubsystem.
   * 
   * @param buttonMonkeyControls
   */
  public ShooterSubsystem() {
    talonFXStarboard = new WPI_TalonFX(Constants.Shooter.kStarboardPort);
    talonFXPort = new WPI_TalonFX(
        Constants.Shooter.kPortPort);

    talonFXStarboard.configFactoryDefault();
    talonFXPort.configFactoryDefault();

    /* Config neutral deadband to be the smallest possible */
    talonFXStarboard.configNeutralDeadband(0.001);
    /* Config sensor used for Primary PID [Velocity] */
    talonFXStarboard.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.Shooter.PID.kPIDLoopIdx,
        Constants.Shooter.PID.kTimeoutMs);

    /* Config the peak and nominal outputs */
    talonFXStarboard.configNominalOutputForward(0, Constants.Shooter.PID.kTimeoutMs);
    talonFXStarboard.configNominalOutputReverse(0, Constants.Shooter.PID.kTimeoutMs);
    talonFXStarboard.configPeakOutputForward(1, Constants.Shooter.PID.kTimeoutMs);
    talonFXStarboard.configPeakOutputReverse(-1, Constants.Shooter.PID.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    talonFXStarboard.config_kF(Constants.Shooter.PID.kPIDLoopIdx, Constants.Shooter.PID.k_Velocity_F,
        Constants.Shooter.PID.kTimeoutMs);
    talonFXStarboard.config_kP(Constants.Shooter.PID.kPIDLoopIdx, Constants.Shooter.PID.k_Velocity_P,
        Constants.Shooter.PID.kTimeoutMs);
    talonFXStarboard.config_kI(Constants.Shooter.PID.kPIDLoopIdx, Constants.Shooter.PID.k_Velocity_I,
        Constants.Shooter.PID.kTimeoutMs);
    talonFXStarboard.config_kD(Constants.Shooter.PID.kPIDLoopIdx, Constants.Shooter.PID.k_Velocity_D,
        Constants.Shooter.PID.kTimeoutMs);

    // /* Config neutral deadband to be the smallest possible */
    // talonFXPort.configNeutralDeadband(0.001);
    // /* Config sensor used for Primary PID [Velocity] */
    // talonFXPort.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
    // Constants.Shooter.PID.kPIDLoopIdx,
    // Constants.Shooter.PID.kTimeoutMs);

    // /* Config the peak and nominal outputs */
    // talonFXPort.configNominalOutputForward(0, Constants.Shooter.PID.kTimeoutMs);
    // talonFXPort.configNominalOutputReverse(0, Constants.Shooter.PID.kTimeoutMs);
    // talonFXPort.configPeakOutputForward(1, Constants.Shooter.PID.kTimeoutMs);
    // talonFXPort.configPeakOutputReverse(-1, Constants.Shooter.PID.kTimeoutMs);

    // /* Config the Velocity closed loop gains in slot0 */
    // talonFXPort.config_kF(Constants.Shooter.PID.kPIDLoopIdx,
    // Constants.Shooter.PID.k_Velocity_F,
    // Constants.Shooter.PID.kTimeoutMs);
    // talonFXPort.config_kP(Constants.Shooter.PID.kPIDLoopIdx,
    // Constants.Shooter.PID.k_Velocity_P,
    // Constants.Shooter.PID.kTimeoutMs);
    // talonFXPort.config_kI(Constants.Shooter.PID.kPIDLoopIdx,
    // Constants.Shooter.PID.k_Velocity_I,
    // Constants.Shooter.PID.kTimeoutMs);
    // talonFXPort.config_kD(Constants.Shooter.PID.kPIDLoopIdx,
    // Constants.Shooter.PID.k_Velocity_D,
    // Constants.Shooter.PID.kTimeoutMs);

    talonFXStarboard.setInverted(false);
    talonFXPort.follow(talonFXStarboard);
    talonFXPort.setInverted(InvertType.OpposeMaster);
    talonFXStarboard.setNeutralMode(NeutralMode.Coast);
    talonFXPort.setNeutralMode(NeutralMode.Coast);

    /* Factory Default all hardware to prevent unexpected behaviour */

    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is
     * integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     * 
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
     * sensor-phase
     */
    // _talon.setSensorPhase(true);
    talonFXStarboard.set(ControlMode.PercentOutput, 0);
    // talonFXPort.set(ControlMode.PercentOutput, 0);

    PhysicsSim.getInstance().addTalonFX(talonFXStarboard, 0.75, 20660);
    PhysicsSim.getInstance().addTalonFX(talonFXPort, 0.75, 20660);

    // SmartDashboard.putNumber("RPMSelect", rpmSpeedSelect);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // rpmSpeedSelect = SmartDashboard.getNumber("RPMSelect", 4650);

    double shooterRPM = talonFXStarboard.getSelectedSensorVelocity() /
    Constants.Shooter.kShooterRPMToTicksPerDS;

    SmartDashboard.putNumber("ShooterSpeed(RPM)", shooterRPM);
  }



  public void setSpeedFar() {
    this.shooterSpeed = Constants.Shooter.kFarShotRPM * Constants.Shooter.kShooterRPMToTicksPerDS;
    talonFXStarboard.set(ControlMode.Velocity, this.shooterSpeed);
    talonFXPort.set(ControlMode.Follower, 20);
  }

  public void setDefaultSpeed() {
    this.shooterSpeed = 1500 * Constants.Shooter.kShooterRPMToTicksPerDS;
    talonFXStarboard.set(ControlMode.Velocity, this.shooterSpeed);
    talonFXPort.set(ControlMode.Follower, 20);
  }

  public void setSpeedDialed(double RPM) {
    this.shooterSpeed = RPM * Constants.Shooter.kShooterRPMToTicksPerDS;
    talonFXStarboard.set(ControlMode.Velocity, this.shooterSpeed);
    talonFXPort.set(ControlMode.Follower, 20);
  }

  public void setSpeedLow() {
    this.shooterSpeed = Constants.Shooter.kLowShotRPM * Constants.Shooter.kShooterRPMToTicksPerDS;
    talonFXStarboard.set(ControlMode.Velocity, this.shooterSpeed);
    talonFXPort.set(ControlMode.Follower, 20);
  }

  public void setSpeedHigh() {
    this.shooterSpeed = Constants.Shooter.kHighShotRPM * Constants.Shooter.kShooterRPMToTicksPerDS;
    talonFXStarboard.set(ControlMode.Velocity, this.shooterSpeed);
    talonFXPort.set(ControlMode.Follower, 20);
  }

  public void shootReverse() {
    this.shooterSpeed = Constants.Shooter.kLowShotRPM * Constants.Shooter.kShooterRPMToTicksPerDS;
    talonFXStarboard.set(ControlMode.Velocity, -this.shooterSpeed);
    talonFXPort.set(ControlMode.Follower, 20);
  }

  public double getSpeed() {
    return talonFXStarboard.getSelectedSensorVelocity() / Constants.Shooter.kShooterRPMToTicksPerDS;
  }

  public double getTickPerDS() {
    return (talonFXStarboard.getSensorCollection().getIntegratedSensorVelocity());
  }

  public void Off() {
    talonFXStarboard.set(ControlMode.PercentOutput, 0);
  }

  public void setShooterSpeed(double shooterSpeed) {
    this.shooterSpeed = shooterSpeed * Constants.Shooter.kShooterRPMToTicksPerDS;
    talonFXStarboard.set(ControlMode.Velocity, this.shooterSpeed);
    talonFXPort.set(ControlMode.Follower, 20);
  }
}
