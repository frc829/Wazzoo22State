// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.HighShot;
import frc.robot.commands.Load;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.PincerSubsystem;
import frc.robot.subsystems.PoweredHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Norm extends SequentialCommandGroup {
  /** Creates a new OldFaithful. */

  public Norm(
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      LifterSubsystem lifterSubsystem,
      SingulatorSubsystem singulatorSubsystem,
      ShooterSubsystem shooterSubsystem,
      PincerSubsystem pincerSubsystem, 
      PoweredHoodSubsystem poweredHoodSubsystem) {



    Load load = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
    ParallelRaceGroup shootHigh = new HighShot(shooterSubsystem, singulatorSubsystem, lifterSubsystem, intakeSubsystem,pincerSubsystem, poweredHoodSubsystem ).withTimeout(1);
    InstantCommand resetGyro = new InstantCommand(() -> driveSubsystem.resetGyro(), driveSubsystem);
    InstantCommand resetOdometry = new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d()), driveSubsystem);
    InstantCommand setFieldCentric = new InstantCommand(() -> driveSubsystem.setFieldCentric(true), driveSubsystem);
    DriveAuto goForward = new DriveAuto(
        new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
        driveSubsystem);
    SequentialCommandGroup path = new SequentialCommandGroup(goForward);
    ParallelRaceGroup travelPathAndLoad = new ParallelRaceGroup(
      load, 
      path);

    addCommands(
        resetGyro,
        resetOdometry,
        setFieldCentric,
        shootHigh, 
        travelPathAndLoad);
  }
}
