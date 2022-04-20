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
import frc.robot.commands.DriveAutoFasterLinearSpeed;
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
public class OldFaithful extends SequentialCommandGroup {
        /** Creates a new OldFaithful. */

        public OldFaithful(
                        DriveSubsystem driveSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        LifterSubsystem lifterSubsystem,
                        SingulatorSubsystem singulatorSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        PincerSubsystem pincerSubsystem,
                        PoweredHoodSubsystem poweredHoodSubsystem) {

                InstantCommand resetGyro = new InstantCommand(() -> driveSubsystem.resetGyro(), driveSubsystem);
                InstantCommand resetOdometry = new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d()),
                                driveSubsystem);
                InstantCommand setFieldCentric = new InstantCommand(() -> driveSubsystem.setFieldCentric(true),
                                driveSubsystem);
                InstantCommand setAbsolute = new InstantCommand(() -> driveSubsystem
                                .setAbsoluteOdometry(new Pose2d(7.651, 1.821, Rotation2d.fromDegrees(-90))),
                                driveSubsystem);

                Load load1 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                RunCommand charge1 = new RunCommand(() -> shooterSubsystem.setSpeedDialed(2900), shooterSubsystem);

                RunCommand stopDrive1 = new RunCommand(() -> driveSubsystem.stopDrive(), driveSubsystem);

                WaitCommand killTravel1 = new WaitCommand(2);

                WaitCommand killShoot1 = new WaitCommand(15);

                FarShotDialedRPM shooter1 = new FarShotDialedRPM(shooterSubsystem, singulatorSubsystem,
                                lifterSubsystem, intakeSubsystem, pincerSubsystem, poweredHoodSubsystem, 2900);

                DriveAutoFasterLinearSpeed path1 = new DriveAutoFasterLinearSpeed(
                                new Pose2d(1.06, 0, Rotation2d.fromDegrees(-13)),
                                driveSubsystem);

                ParallelRaceGroup grab2 = new ParallelRaceGroup(charge1, load1, path1, killTravel1);

                ParallelRaceGroup shoot12 = new ParallelRaceGroup(shooter1, stopDrive1, killShoot1);

                addCommands(
                                resetGyro,
                                resetOdometry,
                                setFieldCentric,
                                setAbsolute,
                                grab2,
                                shoot12);
        }
}
