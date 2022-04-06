// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveManual extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final XboxController driverControls;

  /**
   * Creates a new SwerveModuleDrive.
   * 
   * @param driverControls2
   * @param driveSubsystem2
   */
  public DriveManual(DriveSubsystem driveSubsystem, XboxController driverControls) {

    this.driveSubsystem = driveSubsystem;
    this.driverControls = driverControls;
    addRequirements(driveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    BooleanSupplier buttonAPressed = () -> this.driverControls.getAButton();
    BooleanSupplier buttonYPressed = () -> this.driverControls.getYButton();
    BooleanSupplier buttonRBPressed = () -> this.driverControls.getRightBumper();
    DoubleSupplier leftXAxis = () -> -this.driverControls.getLeftX();
    DoubleSupplier leftYAxis = () -> -this.driverControls.getLeftY();
    DoubleSupplier rightXAxis = () -> -this.driverControls.getRightX();

    // Enable Field Centric Driving
    if (buttonAPressed.getAsBoolean()) {
      driveSubsystem.resetGyro();
      driveSubsystem.setFieldCentric(true);
    }

    // Disable Field Centric Driving
    if (buttonYPressed.getAsBoolean()) {
      driveSubsystem.resetGyro();
      driveSubsystem.setFieldCentric(false);
    }

    //Temporary Disable Field Centric Driving
    if(buttonRBPressed.getAsBoolean()){
      driveSubsystem.setFieldCentricOverride(true);
    }
    else{
      driveSubsystem.setFieldCentricOverride(false);
    }




    double vxMetersPerSecond = Constants.SwerveDrive.General.kMaxSpeedMetersPerSecond * leftYAxis.getAsDouble();
    double vyMetersPerSecond = Constants.SwerveDrive.General.kMaxSpeedMetersPerSecond * leftXAxis.getAsDouble();
    double omegaRadiansPerSecond = Constants.SwerveDrive.General.kMaxAngularSpeedRadiansPerSecond * rightXAxis.getAsDouble();

    double directionalAngle = Math.atan2(vyMetersPerSecond, vxMetersPerSecond);
    double vMetersPerSecond = Math.sqrt(vxMetersPerSecond * vxMetersPerSecond + vyMetersPerSecond * vyMetersPerSecond);

    vMetersPerSecond = DriveSubsystem.adjustVelocity(vMetersPerSecond);
    omegaRadiansPerSecond = DriveSubsystem.adjustOmega(omegaRadiansPerSecond);

    if (vMetersPerSecond == 0 && omegaRadiansPerSecond == 0) {
      driveSubsystem.stopDrive();
    } else {
      vxMetersPerSecond = vMetersPerSecond * Math.cos(directionalAngle);
      vyMetersPerSecond = vMetersPerSecond * Math.sin(directionalAngle);

      SwerveModuleState[] swerveModuleStates = driveSubsystem.GetModuleStates(
          vxMetersPerSecond,
          vyMetersPerSecond,
          omegaRadiansPerSecond);
      

      driveSubsystem.setSwerveModules(swerveModuleStates);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
