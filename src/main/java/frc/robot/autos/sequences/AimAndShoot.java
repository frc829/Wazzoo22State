// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.sequences;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChargeShootFromDistanceAndStop;
import frc.robot.commands.ChargeShootFromDistanceForever;
import frc.robot.commands.DriveAim;
import frc.robot.commands.ShootFromDistance;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PincerSubsystem;
import frc.robot.subsystems.PoweredHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;

public class AimAndShoot extends SequentialCommandGroup {

  public AimAndShoot(
      DriveSubsystem driveSubsystem,
      LimelightSubsystem limelightSubsystem,
      SingulatorSubsystem singulatorSubsystem,
      LifterSubsystem lifterSubsystem,
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      PincerSubsystem pincerSubsystem,
      PoweredHoodSubsystem poweredHoodSubsystem) {
    DriveAim aim = new DriveAim(driveSubsystem, limelightSubsystem);

    ChargeShootFromDistanceForever chargeShoot1 = new ChargeShootFromDistanceForever(
        shooterSubsystem,
        limelightSubsystem,
        pincerSubsystem,
        poweredHoodSubsystem,
        lifterSubsystem,
        intakeSubsystem,
        singulatorSubsystem);
    ChargeShootFromDistanceAndStop chargeShoot2 = new ChargeShootFromDistanceAndStop(
        shooterSubsystem,
        limelightSubsystem,
        pincerSubsystem,
        poweredHoodSubsystem,
        lifterSubsystem,
        intakeSubsystem,
        singulatorSubsystem);
    ShootFromDistance shoot = new ShootFromDistance(
        shooterSubsystem,
        limelightSubsystem,
        pincerSubsystem,
        poweredHoodSubsystem,
        lifterSubsystem,
        intakeSubsystem,
        singulatorSubsystem);
    ParallelRaceGroup aimAndChargeShooter = new ParallelRaceGroup(aim, chargeShoot1);
    // InstantCommand shooterOff = new InstantCommand(() -> shooterSubsystem.Off(),
    // shooterSubsystem);
    addCommands(aimAndChargeShooter, chargeShoot2, shoot);
  }
}
