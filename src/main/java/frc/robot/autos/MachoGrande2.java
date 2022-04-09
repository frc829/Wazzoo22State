// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.DriveAuto2;
import frc.robot.commands.DriveAuto3;
import frc.robot.commands.DriveAuto5;
import frc.robot.commands.DriveAuto6;
import frc.robot.commands.FarShotDialedRPM;
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
public class MachoGrande2 extends SequentialCommandGroup {
        /** Creates a new OldFaithful. */

        public MachoGrande2(
                        DriveSubsystem driveSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        LifterSubsystem lifterSubsystem,
                        SingulatorSubsystem singulatorSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        PincerSubsystem pincerSubsystem,
                        PoweredHoodSubsystem poweredHoodSubsystem) {

                Load load1 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                Load load2 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                Load load3 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                RunCommand charge1 = new RunCommand(() -> shooterSubsystem.setSpeedDialed(2900), shooterSubsystem);
                RunCommand charge2 = new RunCommand(() -> shooterSubsystem.setSpeedDialed(2900), shooterSubsystem);
                RunCommand charge3 = new RunCommand(() -> shooterSubsystem.setSpeedDialed(3600), shooterSubsystem);
                ParallelRaceGroup shootHigh1 = new FarShotDialedRPM(shooterSubsystem, singulatorSubsystem,
                                lifterSubsystem, intakeSubsystem, pincerSubsystem, poweredHoodSubsystem, 2900)
                                                .withTimeout(0.92);
                InstantCommand turnShooterOn = new InstantCommand(() -> shooterSubsystem.setSpeedFar(),
                                driveSubsystem);
                ParallelRaceGroup shootHigh2 = new FarShotDialedRPM(shooterSubsystem, singulatorSubsystem,
                                lifterSubsystem, intakeSubsystem, pincerSubsystem, poweredHoodSubsystem, 2900)
                                                .withTimeout(0.6);
                ParallelRaceGroup shootHigh3 = new FarShotDialedRPM(shooterSubsystem, singulatorSubsystem,
                                lifterSubsystem, intakeSubsystem, pincerSubsystem, poweredHoodSubsystem, 3600)
                                                .withTimeout(1);
                InstantCommand resetGyro = new InstantCommand(() -> driveSubsystem.resetGyro(), driveSubsystem);
                InstantCommand resetOdometry = new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d()),
                                driveSubsystem);
                InstantCommand setFieldCentric = new InstantCommand(() -> driveSubsystem.setFieldCentric(true),
                                driveSubsystem);
                InstantCommand setAbsolute = new InstantCommand(() -> driveSubsystem
                                .setAbsoluteOdometry(new Pose2d(7.651, 1.821, Rotation2d.fromDegrees(-90))),
                                driveSubsystem);

                DriveAuto grab3 = new DriveAuto(
                                new Pose2d(1.05, 0, Rotation2d.fromDegrees(-13)),
                                driveSubsystem);

                DriveAuto grab2 = new DriveAuto(
                                new Pose2d(1.05, 0, Rotation2d.fromDegrees(-75)),
                                driveSubsystem);
                DriveAuto6 goToGoal1 = new DriveAuto6(
                                new Pose2d(-0.25, -2.62, Rotation2d.fromDegrees(-75)),
                                driveSubsystem);
                DriveAuto5 goToGoal2 = new DriveAuto5(
                                new Pose2d(-0.25, -2.62, Rotation2d.fromDegrees(-55)),
                                driveSubsystem);

                DriveAuto2 goToGoal3 = new DriveAuto2(
                                new Pose2d(-0.15, -6.98, Rotation2d.fromDegrees(-55)),
                                driveSubsystem);
                // DriveAuto goToGoal4 = new DriveAuto(
                // new Pose2d(-0.2, -2.70, Rotation2d.fromDegrees(-50)),
                // driveSubsystem);

                DriveAuto2 goToGoal4 = new DriveAuto2(
                                new Pose2d(-2.006, -3.785, Rotation2d.fromDegrees(-50)),
                                driveSubsystem);

                // DriveAuto goToGoal5 = new DriveAuto(
                // new Pose2d(-0.2, -2.70, Rotation2d.fromDegrees(-50)),
                // driveSubsystem);

                DriveAuto goToGoal5 = new DriveAuto(
                                new Pose2d(-0.5, -5.0, Rotation2d.fromDegrees(-50)),
                                driveSubsystem);

                SequentialCommandGroup path1 = new SequentialCommandGroup(
                                // grab2
                                // ,
                                grab3);
                ParallelRaceGroup travelPathAndLoad1 = new ParallelRaceGroup(
                                charge1,
                                load1,
                                path1);

                SequentialCommandGroup path2 = new SequentialCommandGroup(grab2, goToGoal1, goToGoal2);
                ParallelRaceGroup travelPathAndLoad2 = new ParallelRaceGroup(
                                charge2,
                                load2,
                                path2);

                WaitCommand waitCommand = new WaitCommand(0);

                SequentialCommandGroup path3 = new SequentialCommandGroup(
                                goToGoal3,
                                // waitCommand
                                // ,
                                // goToGoal4
                                // ,
                                goToGoal5);
                ParallelRaceGroup travelPathAndLoad3 = new ParallelRaceGroup(
                                charge3,
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
