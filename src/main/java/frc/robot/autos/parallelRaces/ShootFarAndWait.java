// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.parallelRaces;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FarShot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.PincerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootFarAndWait extends ParallelRaceGroup {
  /** Creates a new OldFaithful. */

  public ShootFarAndWait(
      ShooterSubsystem shooterSubsystem,
      SingulatorSubsystem singulatorSubsystem, 
      LifterSubsystem lifterSubsystem, 
      IntakeSubsystem intakeSubsystem,
      PincerSubsystem pincerSubsystem) {
        WaitCommand wait2 = new WaitCommand(1);
        FarShot farShot = new FarShot(shooterSubsystem, singulatorSubsystem, lifterSubsystem, intakeSubsystem, pincerSubsystem);

    addCommands(farShot, wait2);
  }
}
