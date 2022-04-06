// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;


public class DriveAim extends CommandBase {
  private final DriveSubsystem driveSubSystem;
  private final LimelightSubsystem limelightSubsystem;
  private final PIDController thetaController;

  private double turnAngle = 0;
  private double gyroAngle;
  private double turnAngleDegrees = 0;
  private double oldGyroAngle = 0;

  public DriveAim(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {

    this.thetaController = new PIDController(0.12, 0, 0);
    thetaController.enableContinuousInput(-180, 180);
    thetaController.setTolerance(0.25);

    this.driveSubSystem = driveSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Drive Stopped by Auto", false);

    this.turnAngle = limelightSubsystem.getTurnAngle();
    this.oldGyroAngle = driveSubSystem.getGyro();
    this.driveSubSystem.resetGyro();

    this.gyroAngle = driveSubSystem.getGyro();
    double turnAngleDegrees = turnAngle;
    double gyroAngleDegrees = gyroAngle;
    this.turnAngleDegrees = gyroAngleDegrees + turnAngleDegrees;

    this.thetaController.setSetpoint(-turnAngleDegrees);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("TurnAngle", this.turnAngle);
    SmartDashboard.putNumber("GyroAngle", this.gyroAngle);
    SmartDashboard.putNumber("TargetAngle", this.turnAngleDegrees);

    double gyroAngle = driveSubSystem.getGyro();
    double omegaRadiansPerSecond = thetaController.calculate(gyroAngle);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
        omegaRadiansPerSecond, Rotation2d.fromDegrees(gyroAngle));
    SwerveModuleState[] states = Constants.SwerveDrive.Kinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    this.driveSubSystem.setSwerveModules(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveSubSystem.stopDrive();

    if (this.driveSubSystem.isFieldCentric()) {
      Rotation2d old = Rotation2d.fromDegrees(this.oldGyroAngle);
      Rotation2d current = Rotation2d.fromDegrees(this.driveSubSystem.getGyro());
      Rotation2d resetAngle = old.plus(current);
      this.driveSubSystem.resetGyro();
      this.driveSubSystem.setGyroAdjustment(-resetAngle.getDegrees());
    }

    SmartDashboard.putBoolean("Drive Stopped by Auto", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (thetaController.atSetpoint());
  }
}
