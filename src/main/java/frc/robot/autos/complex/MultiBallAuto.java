// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.complex;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.parallelRaces.PathAndLoad;
import frc.robot.autos.parallelRaces.ShootHighAndWait;
import frc.robot.autos.sequences.OdometryInit;
import frc.robot.autos.sequences.TravelPath;
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
public class MultiBallAuto extends SequentialCommandGroup {

    public MultiBallAuto(
            DriveSubsystem driveSubsystem,
            IntakeSubsystem intakeSubsystem,
            LifterSubsystem lifterSubsystem,
            SingulatorSubsystem singulatorSubsystem,
            ShooterSubsystem shooterSubsystem,
            PincerSubsystem pincerSubsystem,
            PoweredHoodSubsystem poweredHoodSubsystem,
            Pose2d startingPose,
            ArrayList<Pose2d> travel1,
            boolean isShooting1,
            ArrayList<Pose2d> travel2,
            boolean isShooting2,
            ArrayList<Pose2d> travel3,
            boolean isShooting3,
            ArrayList<Pose2d> travel4,
            boolean isShooting4,
            ArrayList<Pose2d> travel5,
            boolean isShooting5) {

        OdometryInit odometryInit = new OdometryInit(driveSubsystem);

        TravelPath travelPath1 = new TravelPath(driveSubsystem, startingPose, travel1);
        PathAndLoad pathAndLoad1 = new PathAndLoad(driveSubsystem, intakeSubsystem, lifterSubsystem,
                singulatorSubsystem,
                travelPath1);

        TravelPath travelPath2 = new TravelPath(driveSubsystem, startingPose, travel2);
        PathAndLoad pathAndLoad2 = new PathAndLoad(driveSubsystem, intakeSubsystem, lifterSubsystem,
                singulatorSubsystem,
                travelPath2);

        TravelPath travelPath3 = new TravelPath(driveSubsystem, startingPose, travel3);
        PathAndLoad pathAndLoad3 = new PathAndLoad(driveSubsystem, intakeSubsystem, lifterSubsystem,
                singulatorSubsystem,
                travelPath3);

        TravelPath travelPath4 = new TravelPath(driveSubsystem, startingPose, travel4);
        PathAndLoad pathAndLoad4 = new PathAndLoad(driveSubsystem, intakeSubsystem, lifterSubsystem,
                singulatorSubsystem,
                travelPath4);

        TravelPath travelPath5 = new TravelPath(driveSubsystem, startingPose, travel2);
        PathAndLoad pathAndLoad5 = new PathAndLoad(driveSubsystem, intakeSubsystem, lifterSubsystem,
                singulatorSubsystem,
                travelPath5);

        ShootHighAndWait shoot1 = new ShootHighAndWait(shooterSubsystem, singulatorSubsystem, lifterSubsystem,
                intakeSubsystem, pincerSubsystem, poweredHoodSubsystem);
        ShootHighAndWait shoot2 = new ShootHighAndWait(shooterSubsystem, singulatorSubsystem, lifterSubsystem,
                intakeSubsystem, pincerSubsystem, poweredHoodSubsystem);
        ShootHighAndWait shoot3 = new ShootHighAndWait(shooterSubsystem, singulatorSubsystem, lifterSubsystem,
                intakeSubsystem, pincerSubsystem, poweredHoodSubsystem);
        ShootHighAndWait shoot4 = new ShootHighAndWait(shooterSubsystem, singulatorSubsystem, lifterSubsystem,
                intakeSubsystem, pincerSubsystem, poweredHoodSubsystem);
        ShootHighAndWait shoot5 = new ShootHighAndWait(shooterSubsystem, singulatorSubsystem, lifterSubsystem,
                intakeSubsystem, pincerSubsystem, poweredHoodSubsystem);

        addCommands(
                odometryInit);

        if (isShooting1) {
            addCommands(
                    pathAndLoad1,
                    shoot1);
        }
        if (isShooting2) {
            addCommands(
                    pathAndLoad2,
                    shoot2);
        }
        if (isShooting3) {
            addCommands(
                    pathAndLoad3,
                    shoot3);
        }
        if (isShooting4) {
            addCommands(
                    pathAndLoad4,
                    shoot4);
        }
        if (isShooting5) {
            addCommands(
                    pathAndLoad5,
                    shoot5);
        }
    }
}
