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
import frc.robot.autos.sequences.AimAndShoot;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.FarShotDialedRPM;
import frc.robot.commands.Load;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PincerSubsystem;
import frc.robot.subsystems.PoweredHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GawrshWithLimeLight extends SequentialCommandGroup {

        public GawrshWithLimeLight(
                        DriveSubsystem driveSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        LifterSubsystem lifterSubsystem,
                        SingulatorSubsystem singulatorSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        PincerSubsystem pincerSubsystem,
                        PoweredHoodSubsystem poweredHoodSubsystem, 
                        LimelightSubsystem limelightSubsystem) {

                InstantCommand resetGyro = new InstantCommand(() -> driveSubsystem.resetGyro(), driveSubsystem);
                InstantCommand resetOdometry = new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d()),
                                driveSubsystem);
                InstantCommand setFieldCentric = new InstantCommand(() -> driveSubsystem.setFieldCentric(true),
                                driveSubsystem);
                InstantCommand setAbsolute = new InstantCommand(() -> driveSubsystem
                                .setAbsoluteOdometry(new Pose2d(7.651, 1.821, Rotation2d.fromDegrees(-90))),
                                driveSubsystem);

                Load load1 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                Load load2 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                Load load3 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                Load load4 = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
                RunCommand charge1 = new RunCommand(() -> shooterSubsystem.setSpeedDialed(2900), shooterSubsystem);
                RunCommand charge2 = new RunCommand(() -> shooterSubsystem.setSpeedDialed(2900), shooterSubsystem);
                RunCommand charge3 = new RunCommand(() -> shooterSubsystem.setSpeedDialed(2900), shooterSubsystem);
                RunCommand charge4 = new RunCommand(() -> shooterSubsystem.setSpeedDialed(2900), shooterSubsystem);

                RunCommand stopDrive1 = new RunCommand(() -> driveSubsystem.stopDrive(), driveSubsystem);

                WaitCommand killTravel1 = new WaitCommand(2);
                WaitCommand killTravel2 = new WaitCommand(2);
                WaitCommand killTravel3 = new WaitCommand(2);
                WaitCommand killTravel4 = new WaitCommand(2);

                WaitCommand killShoot1 = new WaitCommand(1.2);

                FarShotDialedRPM shooter1 = new FarShotDialedRPM(shooterSubsystem, singulatorSubsystem,
                                lifterSubsystem, intakeSubsystem, pincerSubsystem, poweredHoodSubsystem, 2900);


                DriveAuto path1 = new DriveAuto(
                                new Pose2d(2.2, 0, Rotation2d.fromDegrees(0)),
                                driveSubsystem);

                DriveAuto path2 = new DriveAuto(
                                new Pose2d(2.2, 0, Rotation2d.fromDegrees(40)),
                                driveSubsystem);

                DriveAuto path3 = new DriveAuto(
                                new Pose2d(2.2, 0, Rotation2d.fromDegrees(70)),
                                driveSubsystem);

                DriveAuto path4 = new DriveAuto(
                                new Pose2d(0, 3, Rotation2d.fromDegrees(70)),
                                driveSubsystem);

                ParallelRaceGroup grab2 = new ParallelRaceGroup(charge1, load1, path1, killTravel1);
                ParallelRaceGroup goToShoot2 = new ParallelRaceGroup(charge2, load2, path2, killTravel2);
                ParallelRaceGroup turnTo3 = new ParallelRaceGroup(charge3, load3, path3, killTravel3);
                ParallelRaceGroup grab3 = new ParallelRaceGroup(charge4, load4, path4, killTravel4);

                ParallelRaceGroup shoot12 = new ParallelRaceGroup(shooter1, stopDrive1, killShoot1);
                AimAndShoot aimAndShoot = new AimAndShoot(driveSubsystem, limelightSubsystem, singulatorSubsystem, lifterSubsystem, intakeSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem);


                addCommands(
                                resetGyro,
                                resetOdometry,
                                setFieldCentric,
                                setAbsolute,
                                grab2,
                                goToShoot2,
                                shoot12,
                                turnTo3,
                                grab3,
                                aimAndShoot);
        }

}
