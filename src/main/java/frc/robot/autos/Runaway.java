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
import frc.robot.commands.DriveAuto3;
import frc.robot.commands.DriveAuto4;
import frc.robot.commands.FarShot;
import frc.robot.commands.Load;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.PincerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Runaway extends SequentialCommandGroup {
        /** Creates a new OldFaithful. */

        public Runaway(
                        DriveSubsystem driveSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        LifterSubsystem lifterSubsystem,
                        SingulatorSubsystem singulatorSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        PincerSubsystem pincerSubsystem) {

                Load load1 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                Load load2 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                Load load3 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                ParallelRaceGroup shootHigh1 = new FarShot(shooterSubsystem, singulatorSubsystem, lifterSubsystem,
                                intakeSubsystem, pincerSubsystem)
                                                .withTimeout(1.5);
                InstantCommand turnShooterOn = new InstantCommand(() -> shooterSubsystem.setSpeedFar(),
                                driveSubsystem);
                ParallelRaceGroup shootHigh2 = new FarShot(shooterSubsystem, singulatorSubsystem, lifterSubsystem,
                                intakeSubsystem, pincerSubsystem).withTimeout(1.5);
                ParallelRaceGroup shootHigh3 = new FarShot(shooterSubsystem, singulatorSubsystem, lifterSubsystem,
                                intakeSubsystem, pincerSubsystem).withTimeout(1.5);
                InstantCommand resetGyro = new InstantCommand(() -> driveSubsystem.resetGyro(), driveSubsystem);
                InstantCommand resetOdometry = new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d()),
                                driveSubsystem);
                InstantCommand setFieldCentric = new InstantCommand(() -> driveSubsystem.setFieldCentric(true),
                                driveSubsystem);
                InstantCommand setAbsolute = new InstantCommand(() -> driveSubsystem
                                .setAbsoluteOdometry(new Pose2d(7.651, 1.821, Rotation2d.fromDegrees(-90))),
                                driveSubsystem);

                DriveAuto grab2 = new DriveAuto(
                                new Pose2d(1.05, 0, Rotation2d.fromDegrees(0)),
                                driveSubsystem);
                DriveAuto grab3 = new DriveAuto(
                                new Pose2d(0.461, 0, Rotation2d.fromDegrees(-13)),
                                driveSubsystem);
                DriveAuto goToGoal1 = new DriveAuto(
                                new Pose2d(0, -2.70, Rotation2d.fromDegrees(-80)),
                                driveSubsystem);
                DriveAuto goToGoal2 = new DriveAuto(
                                new Pose2d(-0.60, -1.50, Rotation2d.fromDegrees(-55)),
                                driveSubsystem);

                DriveAuto goToGoal3 = new DriveAuto(
                                new Pose2d(0.15, -5.80, Rotation2d.fromDegrees(-50)),
                                driveSubsystem);
                DriveAuto goToGoal4 = new DriveAuto(
                                new Pose2d(-0.60, -1.50, Rotation2d.fromDegrees(-55)),
                                driveSubsystem);

                SequentialCommandGroup path1 = new SequentialCommandGroup(grab2, grab3);
                ParallelRaceGroup travelPathAndLoad1 = new ParallelRaceGroup(
                                load1,
                                path1);

                SequentialCommandGroup path2 = new SequentialCommandGroup(goToGoal1, goToGoal2);
                ParallelRaceGroup travelPathAndLoad2 = new ParallelRaceGroup(
                                load2,
                                path2);

                SequentialCommandGroup path3 = new SequentialCommandGroup(goToGoal3, goToGoal4);
                ParallelRaceGroup travelPathAndLoad3 = new ParallelRaceGroup(
                                load3,
                                path3);

                addCommands(
                                resetGyro,
                                resetOdometry,
                                setFieldCentric,
                                setAbsolute,
                                turnShooterOn,
                                travelPathAndLoad1,
                                shootHigh1,
                                travelPathAndLoad2,
                                shootHigh2,
                                travelPathAndLoad3,
                                shootHigh3);
        }
}
