// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.sequences;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAuto;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TravelPath extends SequentialCommandGroup {

    public TravelPath(
            DriveSubsystem driveSubsystem,
            Pose2d startingPose,
            ArrayList<Pose2d> wayPoints) {

        for(int i = 0; i < wayPoints.size(); i++){
                Pose2d adjustedPose = AdjustPose(wayPoints.get(i), startingPose);
                DriveAuto driveToWayPoint = new DriveAuto(adjustedPose, driveSubsystem);
                addCommands(driveToWayPoint);
        }
    }

    private Pose2d AdjustPose(Pose2d poseToAdjust, Pose2d adjustmentPose){
        return poseToAdjust.relativeTo(adjustmentPose);
    }
}
