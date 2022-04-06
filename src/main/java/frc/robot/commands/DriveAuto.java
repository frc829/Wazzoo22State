// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveAuto extends CommandBase {
  private final DriveSubsystem driveSubSystem;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;
  private final Pose2d targetPosition;
  private final Pose2d startingPosition;

  private Pose2d currentPosition;

  private double xOutput = 0;
  private double yOutput = 0;
  private double thetaOutput = 0;

  public DriveAuto(Pose2d targetPosition, DriveSubsystem driveSubsystem) {
    xController = new PIDController(AutoConstants.X.kP, 0, 0);
    xController.setTolerance(0.05);
    yController = new PIDController(AutoConstants.Y.kP, 0, 0);
    yController.setTolerance(0.05);

    this.thetaController = new PIDController(
        AutoConstants.Theta.kP,
        0,
        0);
    thetaController.enableContinuousInput(-180, 180);
    thetaController.setTolerance(1);
    this.driveSubSystem = driveSubsystem;

    this.targetPosition = new Pose2d(-targetPosition.getX(), -targetPosition.getY(), targetPosition.getRotation());
    this.startingPosition = driveSubsystem.getPose();
    this.currentPosition = startingPosition;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Drive Stopped by Auto", false);
    this.xController.setSetpoint(this.targetPosition.getX());
    this.yController.setSetpoint(this.targetPosition.getY());
    this.thetaController.setSetpoint(this.targetPosition.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.currentPosition = this.driveSubSystem.getPose();
    this.xOutput = xController.calculate(this.currentPosition.getX());
    this.yOutput = yController.calculate(this.currentPosition.getY());
    this.thetaOutput = this.thetaController.calculate(this.currentPosition.getRotation().getDegrees());

    SwerveModuleState[] swerveModuleStates = this.driveSubSystem.GetModuleStates(-this.xOutput, -this.yOutput,
        this.thetaOutput);
    this.driveSubSystem.setSwerveModules(swerveModuleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveSubSystem.stopDrive();
    SmartDashboard.putBoolean("Drive Stopped by Auto", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint());
  }
}
