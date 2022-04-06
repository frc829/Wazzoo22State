// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PincerSubsystem;
import frc.robot.subsystems.PoweredHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;

public class ShootFromDistance extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final PincerSubsystem pincerSubsystem;
  private final PoweredHoodSubsystem poweredHoodSubsystem;
  private final LifterSubsystem lifterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final SingulatorSubsystem singulatorSubsystem;

  private double distanceToGoal;
  private double shooterSpeed;

  public ShootFromDistance(
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limelightSubsystem,
      PincerSubsystem pincerSubsystem,
      PoweredHoodSubsystem poweredHoodSubsystem, 
      LifterSubsystem lifterSubsystem, 
      IntakeSubsystem intakeSubsystem, 
      SingulatorSubsystem singulatorSubsystem) {

    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.pincerSubsystem = pincerSubsystem;
    this.poweredHoodSubsystem = poweredHoodSubsystem;
    this.lifterSubsystem = lifterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.singulatorSubsystem = singulatorSubsystem;

    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    distanceToGoal = limelightSubsystem.getGoalDistance();

    // Do some fancy math here to figure out shooter rpm;
    if (limelightSubsystem.getGoalDistance() == 0) {
      this.shooterSpeed = 2900;
    } else {
      this.shooterSpeed = 4500 * (distanceToGoal / 4.50);
    }
    this.pincerSubsystem.PincerOpen();
    this.lifterSubsystem.SpinForwards();
    this.intakeSubsystem.SpinForwards();
    this.poweredHoodSubsystem.SpinForwards();
    this.singulatorSubsystem.SpinForward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this.shooterSpeed = 2900; intake at the wall
    // this.shooterSpeed = 4100; //safe zone distance

    this.lifterSubsystem.SpinForwards();
    this.intakeSubsystem.SpinForwards();
    this.poweredHoodSubsystem.SpinForwards();
    this.singulatorSubsystem.SpinForward();
    shooterSubsystem.setShooterSpeed(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
