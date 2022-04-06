// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;

public class Load extends ParallelCommandGroup {

  public Load(LifterSubsystem lifterSubsystem, IntakeSubsystem intakeSubsystem, SingulatorSubsystem singulatorSubsystem) {
    RunCommand liftIn = (new RunCommand(() -> lifterSubsystem.SpinForwards(), lifterSubsystem));
    RunCommand intakeIn = (new RunCommand(() -> intakeSubsystem.SpinForwards(), intakeSubsystem));
    RunCommand singulatorBack = (new RunCommand(() -> singulatorSubsystem.SpinBackwards(), singulatorSubsystem));
    addCommands(liftIn, intakeIn, singulatorBack);
  }
}
