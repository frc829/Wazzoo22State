// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.sequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ChargeShootFromDistance;
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


public class Aim extends SequentialCommandGroup {

  public Aim(
    DriveSubsystem driveSubsystem, 
    LimelightSubsystem limelightSubsystem,
    SingulatorSubsystem singulatorSubsystem,
    LifterSubsystem lifterSubsystem, 
    IntakeSubsystem intakeSubsystem, 
    ShooterSubsystem shooterSubsystem, 
    PincerSubsystem pincerSubsystem, 
    PoweredHoodSubsystem poweredHoodSubsystem) {
    DriveAim aim = new DriveAim(driveSubsystem, limelightSubsystem);

    ChargeShootFromDistance startShoot = new ChargeShootFromDistance(
      shooterSubsystem, 
      limelightSubsystem, 
      pincerSubsystem, 
      poweredHoodSubsystem, 
      lifterSubsystem, 
      intakeSubsystem, 
      singulatorSubsystem);
    WaitCommand waitCommand1 = new WaitCommand(1);
    WaitCommand waitCommand2 = new WaitCommand(1.5);
    ShootFromDistance shoot = new ShootFromDistance(
      shooterSubsystem, 
      limelightSubsystem, 
      pincerSubsystem, 
      poweredHoodSubsystem, 
      lifterSubsystem, 
      intakeSubsystem,
      singulatorSubsystem);
    ParallelRaceGroup shoot1AndWait = new ParallelRaceGroup(startShoot, waitCommand1);
    ParallelRaceGroup shootandWait = new ParallelRaceGroup(shoot, waitCommand2);
    InstantCommand shooterOff = new InstantCommand(() -> shooterSubsystem.Off(), shooterSubsystem);
    addCommands(aim, shoot1AndWait, shootandWait, shooterOff);
  }
}
