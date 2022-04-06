// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {

  private final NetworkTable table;
  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry ta;
  private final NetworkTableEntry ts;
  private final NetworkTableEntry tv;

  double targetOffsetAngle_Horizontal;
  double targetOffsetAngle_Vertical;
  double targetArea;
  double targetSkew;
  boolean targetFound; 
  boolean limelightOn;

  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    ts = table.getEntry("ts");
    tv = table.getEntry("tv");

    targetArea = ta.getDouble(0);
    targetSkew = ts.getDouble(0);
    targetFound = (tv.getDouble(0) == 0) ? false : true;
    limelightOn = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    targetOffsetAngle_Horizontal = tx.getDouble(0);
    targetOffsetAngle_Vertical = ty.getDouble(0);

    targetArea = ta.getDouble(0);
    targetSkew = ts.getDouble(0);
    targetFound = (tv.getDouble(0) == 0) ? false : true;

    SmartDashboard.putBoolean("LimelightTargetFound", targetFound);
    SmartDashboard.putNumber("DistanceToGoalLimelight", this.getGoalDistance());
    SmartDashboard.putNumber("AngleToTarget", this.getTurnAngle());

  }

  public double getGoalDistance() {
    Rotation2d angleToGoal = Rotation2d.fromDegrees(Constants.LimeLight.MountAngleDegrees + targetOffsetAngle_Vertical);
    double distance = (Constants.LimeLight.GoalHeightMeters - Constants.LimeLight.HeightMeters) / angleToGoal.getTan();
    if(distance == Double.NaN){distance = 0;}
    if(distance == Double.POSITIVE_INFINITY){distance = 0;}
    if(distance == Double.NEGATIVE_INFINITY){distance = 0;}
    return distance;
  }

  public double getTurnAngle() {
    return (this.targetOffsetAngle_Horizontal);

  }

  public double getTargetOffsetAngle_Horizontal() {
    return this.targetOffsetAngle_Horizontal;
  }

  public double getTargetOffsetAngle_Vertical() {
    return this.targetOffsetAngle_Vertical;
  }

  public double getTargetArea() {
    return this.targetArea;
  }

  public double getTargetSkew() {
    return this.targetSkew;
  }

  public boolean getTargetFound(){
    return this.targetFound;
  }

  public void ToggleCameraOnOff(){
    this.limelightOn = (this.limelightOn) ? false : true;
    if(this.limelightOn){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }
    else{
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

  }
}
