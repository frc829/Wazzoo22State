// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.parallelRaces;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.autos.sequences.TravelPath;
import frc.robot.commands.Load;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathAndLoad extends ParallelRaceGroup {
  /** Creates a new OldFaithful. */

  public PathAndLoad(
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      LifterSubsystem lifterSubsystem,
      SingulatorSubsystem singulatorSubsystem,
      TravelPath travelPath) {

    Load load = new Load(lifterSubsystem, intakeSubsystem, singulatorSubsystem);
    addCommands(load, travelPath);
  }
}
