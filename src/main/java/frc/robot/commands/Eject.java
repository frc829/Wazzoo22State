// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.PoweredHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Eject extends ParallelCommandGroup {
  /** Creates a new Eject. */
  public Eject(LifterSubsystem lifterSubsystem, IntakeSubsystem intakeSubsystem, SingulatorSubsystem singulatorSubsystem, ShooterSubsystem shooterSubsystem, PoweredHoodSubsystem poweredHoodSubsystem) {
    RunCommand liftOut = (new RunCommand(() -> lifterSubsystem.SpinBackwards(), lifterSubsystem));
    RunCommand intakeOut = (new RunCommand(() -> intakeSubsystem.SpinBackwards(), intakeSubsystem));
    RunCommand singulatorBack = (new RunCommand(() -> singulatorSubsystem.SpinBackwards(), singulatorSubsystem));
    RunCommand shooterBack = (new RunCommand(() -> shooterSubsystem.shootReverse(), shooterSubsystem));
    RunCommand poweredHoodBack = (new RunCommand(() -> poweredHoodSubsystem.SpinBackwards(), poweredHoodSubsystem));
    addCommands(liftOut, intakeOut, singulatorBack, poweredHoodBack, shooterBack);
  }
}
