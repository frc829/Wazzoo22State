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
public class Mammoth extends SequentialCommandGroup {
        /** Creates a new OldFaithful. */

        public Mammoth(
                        DriveSubsystem driveSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        LifterSubsystem lifterSubsystem,
                        SingulatorSubsystem singulatorSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        PincerSubsystem pincerSubsystem) {

                Load load1 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                ParallelRaceGroup shootHigh1 = new FarShot(shooterSubsystem, singulatorSubsystem, lifterSubsystem,
                                intakeSubsystem, pincerSubsystem)
                                                .withTimeout(1.5);

                InstantCommand resetGyro = new InstantCommand(() -> driveSubsystem.resetGyro(), driveSubsystem);
                InstantCommand resetOdometry = new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d()),
                                driveSubsystem);
                InstantCommand setFieldCentric = new InstantCommand(() -> driveSubsystem.setFieldCentric(true),
                                driveSubsystem);

                DriveAuto grab2 = new DriveAuto(
                                new Pose2d(1.10, 0, Rotation2d.fromDegrees(0)),
                                driveSubsystem);
                DriveAuto grab3 = new DriveAuto(
                                new Pose2d(0.461, 0, Rotation2d.fromDegrees(13)),
                                driveSubsystem);

                SequentialCommandGroup path1 = new SequentialCommandGroup(grab2, grab3);
                ParallelRaceGroup travelPathAndLoad1 = new ParallelRaceGroup(
                                load1,
                                path1);

                

                addCommands(
                                resetGyro,
                                resetOdometry,
                                setFieldCentric,
                                travelPathAndLoad1,
                                shootHigh1);
        }
}
