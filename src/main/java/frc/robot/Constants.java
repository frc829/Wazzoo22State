// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class IntakeArm {
        public static final int kForwardChannel = 5;
    }

    public static final class Pincer {
        public static final int kForwardChannel = 6;
    }

    public static final class Intake {
        public static final int kPort = 21;
        public static final double kSpeed = 0.7;
    }

    public static final class Lifter {
        public static final int kPort = 22;
        public static final double kSpeed = 1;
    }

    public static final class Singulator {
        public static final int kPort = 23;
        public static final double kSpeed = 1;
    }

    public static final class Shooter {
        public static final int kPortPort = 18;
        public static final int kStarboardPort = 20;
        public static final double kHighShot = 0.625;
        public static final double kLowShot = 0.35;

        public static final double kHighShotRPM = 2400;
        public static final double kLowShotRPM = 1400;
        public static final double kFarShotRPM = 3950;

        public static final double kHighShotTicksPerDS = 13900;
        public static final double kLowShotTicksPerDS = 7550;
        // encoder Velocity Conversion Constants
        public static final double kShooterRPMToTicksPerDS = 3.413333333;

        public static final class PID {
            public static final int kPIDLoopIdx = 0;
            public static final int kTimeoutMs = 30;
            public static final double k_Velocity_P = 0.1;
            public static final double k_Velocity_I = 0;
            public static final double k_Velocity_D = 0;
            public static final double k_Velocity_F = 1023.0 / 20660.0;
            public static final double k_Velocity_Iz = 0;
            public static final double k_Velocity_PeakOut = 0;
        }

    }

    public static final class Winch {
        public static final int kPort = 24;
        public static final double kSpeed = 1;
        public static final double kMaxDistance = 500;

        public static final double kP = 0.011;
    }

    public static final class WinchArm {
        public static final int kPort = 25;
        public static final double kUpSpeed = 0.5;
        public static final double kDownSpeed = 0.25;
        public static final double kEncoderValueAtBottom = 0;
        public static final double kEncoderValueAtTop = 100;
        public static final double kP = 0.011;
        public static double kMaxDistance = 500;
    }

    public static final class SwerveDrive {

        public static final class Kinematics {

            public static final double kTrackWidth = 0.530;
            // Distance between centers of right and left wheels on robot
            public static final double kWheelBase = 0.525;
            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                    new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                    new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                    new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        }

        public static final class General {

            public static final double kMaxSpeedMetersPerSecond = 3;
            public static final double kMaxAccelerationMetersPerSecondSquared = 3;

            public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
            public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
            // LeftToRightDistance
            public static double trackWidth = 0.530;
            // FrontToBackDistance
            public static double trackLength = 0.525;

            public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
            public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

            public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
            public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                    kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
            public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
            public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        }

        public static final class Module {

            public static final class General {

                // encoder Velocity Conversion Constants
                public static double MPSToTickPerDS = 4491.438849;
            }

            public static final class PID {
                public static final int kPIDLoopIdx = 0;
                public static final int kTimeoutMs = 30;
                public static final double k_Velocity_P = 0.1;
                public static final double k_Velocity_I = 0.001;
                public static final double k_Velocity_D = 5;
                public static final double k_Velocity_F = 1023.0 / 20660.0;
                public static final double k_Velocity_Iz = 300;
                public static final double k_Velocity_PeakOut = 1.00;
            }

            public static final class FrontLeft {

                public static final class Location {
                    public static final double kX = SwerveDrive.General.trackLength / 2;
                    public static final double kY = SwerveDrive.General.trackWidth / 2;
                }

                public static final class RotationControl {
                    public static final class Ports {
                        public static final int kMotorPort = 17;
                        public static final int kEncoderPort = 33;
                    }

                    public static final class PID {
                        public static final double kP = 0.011;
                        public static final double kI = 0;
                        public static final double kD = 0;
                    }
                }

                public static final class DriveControl {
                    public static final class Ports {
                        public static final int kMotorPort = 13;
                    }

                    public static final class PID {
                        public static final int kPIDLoopIdx = 0;
                        public static final int kTimeoutMs = 30;
                        public static final double k_Velocity_P = 0.1;
                        public static final double k_Velocity_I = 0.001;
                        public static final double k_Velocity_D = 5;
                        public static final double k_Velocity_F = 1023.0 / 20660.0;
                        public static final double k_Velocity_Iz = 300;
                        public static final double k_Velocity_PeakOut = 1.00;
                    }
                }
            }

            public static final class FrontRight {

                public static final class Location {
                    public static final double kX = SwerveDrive.General.trackLength / 2;
                    public static final double kY = -SwerveDrive.General.trackWidth / 2;
                }

                public static final class RotationControl {
                    public static final class Ports {
                        public static final int kMotorPort = 16;
                        public static final int kEncoderPort = 32;
                    }

                    public static final class PID {
                        public static final double kP = 0.011;
                        public static final double kI = 0;
                        public static final double kD = 0;
                    }
                }

                public static final class DriveControl {
                    public static final class Ports {
                        public static final int kMotorPort = 12;
                    }

                    public static final class PID {
                        public static final int kPIDLoopIdx = 0;
                        public static final int kTimeoutMs = 30;
                        public static final double k_Velocity_P = 0.1;
                        public static final double k_Velocity_I = 0.001;
                        public static final double k_Velocity_D = 5;
                        public static final double k_Velocity_F = 1023.0 / 20660.0;
                        public static final double k_Velocity_Iz = 300;
                        public static final double k_Velocity_PeakOut = 1.00;
                    }
                }
            }

            public static final class BackLeft {

                public static final class Location {
                    public static final double kX = -SwerveDrive.General.trackLength / 2;
                    public static final double kY = SwerveDrive.General.trackWidth / 2;
                }

                public static final class RotationControl {
                    public static final class Ports {
                        public static final int kMotorPort = 15;
                        public static final int kEncoderPort = 31;
                    }

                    public static final class PID {
                        public static final double kP = 0.011;
                        public static final double kI = 0;
                        public static final double kD = 0;
                    }
                }

                public static final class DriveControl {
                    public static final class Ports {
                        public static final int kMotorPort = 11;
                    }

                    public static final class PID {
                        public static final int kPIDLoopIdx = 0;
                        public static final int kTimeoutMs = 30;
                        public static final double k_Velocity_P = 0.1;
                        public static final double k_Velocity_I = 0.001;
                        public static final double k_Velocity_D = 5;
                        public static final double k_Velocity_F = 1023.0 / 20660.0;
                        public static final double k_Velocity_Iz = 300;
                        public static final double k_Velocity_PeakOut = 1.00;
                    }
                }
            }

            public static final class BackRight {

                public static final class Location {
                    public static final double kX = -SwerveDrive.General.trackLength / 2;
                    public static final double kY = -SwerveDrive.General.trackWidth / 2;
                }

                public static final class RotationControl {
                    public static final class Ports {
                        public static final int kMotorPort = 14;
                        public static final int kEncoderPort = 30;
                    }

                    public static final class PID {
                        public static final double kP = 0.011;
                        public static final double kI = 0;
                        public static final double kD = 0.0;
                    }
                }

                public static final class DriveControl {
                    public static final class Ports {
                        public static final int kMotorPort = 10;
                    }

                    public static final class PID {
                        public static final int kPIDLoopIdx = 0;
                        public static final int kTimeoutMs = 30;
                        public static final double k_Velocity_P = 0.1;
                        public static final double k_Velocity_I = 0.001;
                        public static final double k_Velocity_D = 5;
                        public static final double k_Velocity_F = 1023.0 / 20660.0;
                        public static final double k_Velocity_Iz = 300;
                        public static final double k_Velocity_PeakOut = 1.00;
                    }
                }
            }
        }
    }

    public static final class OI {
        public static final class Controller {
            public static final int kDriverPort = 0;
            public static final int kOperatorPort = 1;
            public static final double kDeadband = 0.1;
        }

        public static final class Buttons {
            public static final int A = 1;
            public static final int B = 2;
            public static final int X = 3;
            public static final int Y = 4;
            public static final int LB = 5;
            public static final int RB = 6;
            public static final int BACK = 7;
            public static final int START = 8;
            public static final int LS = 9;
            public static final int RS = 10;

        }

        public static final class AXES {
            public static final int LX = 0;
            public static final int LY = 1;
            public static final int LT = 2;
            public static final int RT = 3;
            public static final int RX = 4;
            public static final int RY = 5;
        }
    }

    public static final class AutoConstants {
        public static final double maxSpeed = 3.25;
        public static final double maxSpeed2 = 3.5;

        public static final class X {
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        public static final class Y {
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        public static final class Theta {
            public static final double kP = 0.05;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double afterRotateAdjustDistance = 0.1;
        }

        public static final class Balls{
            public static final class Ball1{
                //X = 7.57, Y = 0.29
                public static final double kX = 6.729;
                public static final double kY = 0.50;
            }

            public static final class Ball2{
                public static final double kX = 5.06;
                public static final double kY = 1.87;
            }

            public static final class Ball3{
                public static final double kX = 4.94;
                public static final double kY = 6.17;
            }

            public static final class Ball4{
                public static final double kX = 1.06;
                public static final double kY = 1.12;
            }            
        }

        public static final class Positions{
            //Robot Chassis from Center to Edge with Buffer
            //14.125" + 3" bumper + 1" buffer
            //18.125" = 0.461m
            public static final class Starting1{
                public static double kX = 6.729;
                public static double kY = 1.821;
                public static Rotation2d kTheta = Rotation2d.fromDegrees(-88.46);
            }

            public static final class Starting2{
                public static double kX = 6.729;
                public static double kY = 2.86;
                public static Rotation2d kTheta = Rotation2d.fromDegrees(-112.65);
            }

            public static final class Starting3{
                public static double kX = 6.79;
                public static double kY = 2.38;
                public static Rotation2d kTheta = Rotation2d.fromDegrees(-133.37);
            }

            public static final class Starting4{
                public static double kX = 6.43;
                public static double kY = 5.54;
                public static Rotation2d kTheta = Rotation2d.fromDegrees(139.85);
            }

            public static final class Starting5{
                public static double kX = 7.07;
                public static double kY = 4.71;
                public static Rotation2d kTheta = Rotation2d.fromDegrees(158.81);
            }

            public static final class Goal{
                public static double kX = 8.325;
                public static double kY = 4.114;
            }
        }
    }

    public static final class LimeLight {
        // how many degrees back is your limelight rotated from perfectly vertical?
        public static final double MountAngleDegrees = 40.0;

        // distance from the center of the Limelight lens to the floor
        public static final double HeightMeters = 0.6096;

        // distance from the target to the floor
        public static final double GoalHeightMeters = 2.64;

    }


    public static final class PoweredHood{
        public static final int kPort = 25;
        //public static final double speed = 0.7;
        public static final double speed = 0.7;

    }
}
