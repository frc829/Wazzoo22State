// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveAuto;
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
public class Gawrsh extends SequentialCommandGroup {
        /** Creates a new OldFaithful. */

        public Gawrsh(
                        DriveSubsystem driveSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        LifterSubsystem lifterSubsystem,
                        SingulatorSubsystem singulatorSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        PincerSubsystem pincerSubsystem,
                        PoweredHoodSubsystem poweredHoodSubsystem) {

                ParallelRaceGroup shootHigh1 = new FarShotDialedRPM(shooterSubsystem, singulatorSubsystem,
                                lifterSubsystem, intakeSubsystem, pincerSubsystem, poweredHoodSubsystem, 2900)
                                                .withTimeout(1.3);

                ParallelRaceGroup shootHigh2 = new FarShotDialedRPM(shooterSubsystem, singulatorSubsystem,
                                lifterSubsystem, intakeSubsystem, pincerSubsystem, poweredHoodSubsystem, 2900)
                                                .withTimeout(1.3);

                Load load1 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                Load load2 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);

                InstantCommand resetGyro = new InstantCommand(() -> driveSubsystem.resetGyro(), driveSubsystem);
                InstantCommand resetOdometry = new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d()),
                                driveSubsystem);
                InstantCommand setFieldCentric = new InstantCommand(() -> driveSubsystem.setFieldCentric(true),
                                driveSubsystem);

                DriveAuto grab2 = new DriveAuto(
                                new Pose2d(2.2, 0, Rotation2d.fromDegrees(0)),
                                driveSubsystem);

                DriveAuto grab = new DriveAuto(
                                new Pose2d(2.2, 0, Rotation2d.fromDegrees(40)),
                                driveSubsystem);
                DriveAuto grab3 = new DriveAuto(
                                new Pose2d(2.2, 0, Rotation2d.fromDegrees(70)),
                                driveSubsystem);

                DriveAuto grab99 = new DriveAuto(
                                new Pose2d(0, 3, Rotation2d.fromDegrees(70)),
                                driveSubsystem);

                DriveAuto grab4 = new DriveAuto(
                                new Pose2d(0, 3, Rotation2d.fromDegrees(70)),
                                driveSubsystem);

                WaitCommand waitCommand = new WaitCommand(2);

                SequentialCommandGroup path1 = new SequentialCommandGroup(grab2, grab);
                SequentialCommandGroup path2 = new SequentialCommandGroup(grab3, grab99, waitCommand, grab4);
                ParallelRaceGroup travelPathAndLoad1 = new ParallelRaceGroup(
                                load1,
                                path1);

                ParallelRaceGroup travelPathAndLoad2 = new ParallelRaceGroup(
                                load2,
                                path2);

                addCommands(
                                resetGyro,
                                resetOdometry,
                                setFieldCentric,
                                travelPathAndLoad1,
                                shootHigh1,
                                travelPathAndLoad2,
                                shootHigh2);
        }
}
