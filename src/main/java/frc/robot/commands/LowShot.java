// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.PincerSubsystem;
import frc.robot.subsystems.PoweredHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;

public class LowShot extends ParallelCommandGroup {

  public LowShot(ShooterSubsystem shooterSubsystem, SingulatorSubsystem singulatorSubsystem,
      LifterSubsystem lifterSubsystem, IntakeSubsystem intakeSubsystem, PincerSubsystem pincerSubsystem,
      PoweredHoodSubsystem poweredHoodSubsystem) {
    InstantCommand chargeLow = new InstantCommand(shooterSubsystem::setSpeedLow, shooterSubsystem);
    InstantCommand powerHood = new InstantCommand(poweredHoodSubsystem::SpinForwards, poweredHoodSubsystem);
    InstantCommand setAngle = new InstantCommand(pincerSubsystem::PincerOpen, pincerSubsystem);
    WaitUntilCommand waitUntilShooterAtSpeed = new WaitUntilCommand(() -> shooterSubsystem
        .getSpeed() >= Constants.Shooter.kHighShotRPM == true);
    RunCommand singulatorFire = new RunCommand(() -> singulatorSubsystem.SpinForward(), singulatorSubsystem);
    RunCommand liftIn = (new RunCommand(() -> lifterSubsystem.SpinForwards(), lifterSubsystem));
    RunCommand intakeIn = (new RunCommand(() -> intakeSubsystem.SpinForwards(), intakeSubsystem));
    SequentialCommandGroup fire = new SequentialCommandGroup(chargeLow, waitUntilShooterAtSpeed, singulatorFire);
    addCommands(setAngle, powerHood, liftIn, intakeIn, fire);
  }
}
