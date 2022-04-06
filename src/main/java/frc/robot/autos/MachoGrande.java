// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAuto2;
import frc.robot.commands.DriveAuto3;
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
public class MachoGrande extends SequentialCommandGroup {
        /** Creates a new OldFaithful. */

        public MachoGrande(
                        DriveSubsystem driveSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        LifterSubsystem lifterSubsystem,
                        SingulatorSubsystem singulatorSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        PincerSubsystem pincerSubsystem) {

                Load load1 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                Load load2 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                ParallelRaceGroup shootHigh1 = new FarShot(shooterSubsystem, singulatorSubsystem, lifterSubsystem,
                                intakeSubsystem, pincerSubsystem)
                                                .withTimeout(0.8);
                ParallelRaceGroup shootHigh2 = new FarShot(shooterSubsystem, singulatorSubsystem, lifterSubsystem,
                                intakeSubsystem, pincerSubsystem)
                                                .withTimeout(1.5);
                InstantCommand resetGyro = new InstantCommand(() -> driveSubsystem.resetGyro(), driveSubsystem);
                InstantCommand resetOdometry = new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d()),
                                driveSubsystem);
                InstantCommand setFieldCentric = new InstantCommand(() -> driveSubsystem.setFieldCentric(true),
                                driveSubsystem);
                InstantCommand turnShooterOn = new InstantCommand(() -> shooterSubsystem.setSpeedFar(),
                                driveSubsystem);

                DriveAuto3 grab2 = new DriveAuto3(
                                new Pose2d(1.40, 0, Rotation2d.fromDegrees(0)),
                                driveSubsystem);
                DriveAuto3 grab3 = new DriveAuto3(
                                new Pose2d(0.461, 0, Rotation2d.fromDegrees(13)),
                                driveSubsystem);
                DriveAuto2 grab4 = new DriveAuto2(
                                new Pose2d(5.24, -1.642, Rotation2d.fromDegrees(15)),
                                driveSubsystem);
                DriveAuto2 grab5 = new DriveAuto2(
                                new Pose2d(0.461, 0, Rotation2d.fromDegrees(13)),
                                driveSubsystem);

                SequentialCommandGroup path1 = new SequentialCommandGroup(grab2, grab3);
                SequentialCommandGroup path2 = new SequentialCommandGroup(grab4, grab5);
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
                                turnShooterOn,
                                travelPathAndLoad1,
                                shootHigh1,
                                travelPathAndLoad2,
                                shootHigh2);
        }
}
