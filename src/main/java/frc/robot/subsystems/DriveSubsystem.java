// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
  private final AHRS gyro;
  private final SwerveModule frontLeftSwerveModule;
  private final SwerveModule frontRightSwerveModule;
  private final SwerveModule backLeftSwerveModule;
  private final SwerveModule backRightSwerveModule;

  private final SwerveDriveOdometry m_odometry;

  private final SwerveDriveOdometry absoluteOdometry;


  private double omega = 0;
  private boolean isFieldCentric = false;
  private boolean fieldCentricOverride = false;
  private double gyroOffset;
  private double theta = 0;

  private double previousTime = 0;

  private SimDouble simGyroAngle;

  public ArrayList<Double> fieldCentricAngles = new ArrayList<Double>();
  private Field2d thefield = new Field2d();
  

  private double absoluteGyroReference = 0;
  private double absoluteGyro = 0;

  private Pose2d startingPose = new Pose2d();
  private Pose2d actualPosition = new Pose2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    this.gyro = new AHRS(SPI.Port.kMXP);

    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    this.simGyroAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

    this.previousTime = Timer.getFPGATimestamp();

    frontLeftSwerveModule = new SwerveModule(
        Constants.SwerveDrive.Module.FrontLeft.RotationControl.PID.kP,
        Constants.SwerveDrive.Module.FrontLeft.RotationControl.PID.kI,
        Constants.SwerveDrive.Module.FrontLeft.RotationControl.PID.kD,
        Constants.SwerveDrive.Module.FrontLeft.RotationControl.Ports.kMotorPort,
        Constants.SwerveDrive.Module.FrontLeft.RotationControl.Ports.kEncoderPort,
        Constants.SwerveDrive.Module.FrontLeft.DriveControl.Ports.kMotorPort);
    frontRightSwerveModule = new SwerveModule(
        Constants.SwerveDrive.Module.FrontRight.RotationControl.PID.kP,
        Constants.SwerveDrive.Module.FrontRight.RotationControl.PID.kI,
        Constants.SwerveDrive.Module.FrontRight.RotationControl.PID.kD,
        Constants.SwerveDrive.Module.FrontRight.RotationControl.Ports.kMotorPort,
        Constants.SwerveDrive.Module.FrontRight.RotationControl.Ports.kEncoderPort,
        Constants.SwerveDrive.Module.FrontRight.DriveControl.Ports.kMotorPort);
    backLeftSwerveModule = new SwerveModule(
        Constants.SwerveDrive.Module.BackLeft.RotationControl.PID.kP,
        Constants.SwerveDrive.Module.BackLeft.RotationControl.PID.kI,
        Constants.SwerveDrive.Module.BackLeft.RotationControl.PID.kD,
        Constants.SwerveDrive.Module.BackLeft.RotationControl.Ports.kMotorPort,
        Constants.SwerveDrive.Module.BackLeft.RotationControl.Ports.kEncoderPort,
        Constants.SwerveDrive.Module.BackLeft.DriveControl.Ports.kMotorPort);
    backRightSwerveModule = new SwerveModule(
        Constants.SwerveDrive.Module.BackRight.RotationControl.PID.kP,
        Constants.SwerveDrive.Module.BackRight.RotationControl.PID.kI,
        Constants.SwerveDrive.Module.BackRight.RotationControl.PID.kD,
        Constants.SwerveDrive.Module.BackRight.RotationControl.Ports.kMotorPort,
        Constants.SwerveDrive.Module.BackRight.RotationControl.Ports.kEncoderPort,
        Constants.SwerveDrive.Module.BackRight.DriveControl.Ports.kMotorPort);

    m_odometry = new SwerveDriveOdometry(
        Constants.SwerveDrive.Kinematics.kDriveKinematics,
        Rotation2d.fromDegrees(getGyro()));

    absoluteOdometry = new SwerveDriveOdometry(
        Constants.SwerveDrive.Kinematics.kDriveKinematics,
        Rotation2d.fromDegrees(getGyro()));

    resetGyro();
    setFieldCentric(false);
  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.absoluteGyro = this.absoluteGyroReference + this.getGyro();
    this.previousTime = Timer.getFPGATimestamp();
    absoluteOdometry.update(
        Rotation2d.fromDegrees(this.absoluteGyro),
        this.frontLeftSwerveModule.getState(),
        this.frontRightSwerveModule.getState(),
        this.backLeftSwerveModule.getState(),
        this.backRightSwerveModule.getState());

    this.actualPosition = new Pose2d(
        this.startingPose.getX() + (-absoluteOdometry.getPoseMeters().getX() + this.startingPose.getX()),
        this.startingPose.getY() + (-absoluteOdometry.getPoseMeters().getY() + this.startingPose.getY()),
        this.absoluteOdometry.getPoseMeters().getRotation());

    thefield.setRobotPose(
        this.actualPosition.getX(),
        this.actualPosition.getY(),
        this.actualPosition.getRotation());

    m_odometry.update(
        Rotation2d.fromDegrees(getGyro()),
        this.frontLeftSwerveModule.getState(),
        this.frontRightSwerveModule.getState(),
        this.backLeftSwerveModule.getState(),
        this.backRightSwerveModule.getState());

    // SmartDashboard.putNumber("Vx(mps)", vx);
    // SmartDashboard.putNumber("Vy(mps)", vy);
    // SmartDashboard.putNumber("W(radps)", this.omega);
    SmartDashboard.putBoolean("Is Field Centric", isFieldCentric);
    // SmartDashboard.putNumber("X(m)", m_odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("Y(m)", m_odometry.getPoseMeters().getY());
    // SmartDashboard.putNumber("T(deg)", m_odometry.getPoseMeters().getRotation().getDegrees());
    // SmartDashboard.putNumberArray(
    // "AbsoluteOdometry",
    // new double[] {
    // this.absoluteOdometry.getPoseMeters().getX(),
    // this.absoluteOdometry.getPoseMeters().getY(),
    // this.absoluteOdometry.getPoseMeters().getRotation().getDegrees()
    // });

    // SmartDashboard.putNumber("SwerveAngles0", this.frontLeftSwerveModule.currentTurnPosition);
    // SmartDashboard.putNumber("SwerveAngles1", this.frontRightSwerveModule.currentTurnPosition);
    // SmartDashboard.putNumber("SwerveAngles2", this.backLeftSwerveModule.currentTurnPosition);
    // SmartDashboard.putNumber("SwerveAngles3", this.backRightSwerveModule.currentTurnPosition);
    // SmartDashboard.putNumber("AbsoluteGyroReference", this.absoluteGyroReference);


  }

  public boolean isFieldCentric() {
    return this.isFieldCentric;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        pose,
        Rotation2d.fromDegrees(getGyro()));
  }

  public void setAbsoluteOdometry(Pose2d pose) {
    this.startingPose = pose;
    absoluteOdometry.resetPosition(
        pose,
        Rotation2d.fromDegrees(this.absoluteGyroReference + this.getGyro()));
    this.actualPosition = new Pose2d(
        this.startingPose.getX() - (absoluteOdometry.getPoseMeters().getX() - this.startingPose.getX()),
        this.startingPose.getY() - (absoluteOdometry.getPoseMeters().getY() - this.startingPose.getY()),
        this.absoluteOdometry.getPoseMeters().getRotation());
  }

  public void initializeAbsoluteGyroRefernce(double startAngle) {
    this.absoluteGyroReference = startAngle;
  }

  public void setFieldCentric(boolean isFieldCentric) {
    this.fieldCentricAngles.add(this.getGyro());
    this.isFieldCentric = isFieldCentric;
  }

  public void setGyroOffset(double gyroOffset) {
    this.gyroOffset = gyroOffset;
  }

  public void setSwerveModules(SwerveModuleState[] swerveModuleStates) {

    swerveModuleStates = DriveSubsystem.OptimizeModuleStates(swerveModuleStates, this.getModuleAngles());
    ChassisSpeeds chassisSpeeds = Constants.SwerveDrive.Kinematics.kDriveKinematics.toChassisSpeeds(
        swerveModuleStates[0],
        swerveModuleStates[1],
        swerveModuleStates[2],
        swerveModuleStates[3]);


    this.omega = chassisSpeeds.omegaRadiansPerSecond;

    double timeDelta = Timer.getFPGATimestamp() - this.previousTime;

    double deltaTheta = (180 / 3.14159) * this.omega * timeDelta;
    this.theta += deltaTheta;
    this.simGyroAngle.set(-this.theta);

    this.frontLeftSwerveModule.setModule(swerveModuleStates[0]);
    this.frontRightSwerveModule.setModule(swerveModuleStates[1]);
    this.backLeftSwerveModule.setModule(swerveModuleStates[2]);
    this.backRightSwerveModule.setModule(swerveModuleStates[3]);
  }

  public void resetGyro() {
    this.absoluteGyroReference = this.absoluteGyroReference + this.getGyro();
    this.gyroOffset = 0;
    this.gyro.reset();
    this.gyro.setAngleAdjustment(0);
  }

  public double getGyro() {
    return -Rotation2d.fromDegrees(this.gyro.getAngle()).getDegrees() + this.gyroOffset;
  }

  public void setGyroAdjustment(double angle) {
    this.gyro.setAngleAdjustment(angle);
  }

  public double[] getModuleAngles() {
    double[] moduleAngles = new double[4];
    moduleAngles[0] = this.frontLeftSwerveModule.getModuleAngle();
    moduleAngles[1] = this.frontRightSwerveModule.getModuleAngle();
    moduleAngles[2] = this.backLeftSwerveModule.getModuleAngle();
    moduleAngles[3] = this.backRightSwerveModule.getModuleAngle();
    return moduleAngles;
  }

  public void stopDrive() {
    this.frontLeftSwerveModule.turnOffModule();
    this.frontRightSwerveModule.turnOffModule();
    this.backLeftSwerveModule.turnOffModule();
    this.backRightSwerveModule.turnOffModule();

  }

  public static double adjustVelocity(double velocity) {
    if (velocity <= Constants.OI.Controller.kDeadband * Constants.SwerveDrive.General.kMaxSpeedMetersPerSecond) {
      return 0;
    } else {
      return velocity;
    }
  }

  public static double adjustOmega(double omega) {
    if (Math.abs(omega) <= Constants.OI.Controller.kDeadband
        * Constants.SwerveDrive.General.kMaxAngularSpeedRadiansPerSecond) {
      return 0;
    } else {
      double signOfOmega = Math.signum(omega);
      double valueOfOmega = Math.abs(omega);
      valueOfOmega -= Constants.OI.Controller.kDeadband
          * Constants.SwerveDrive.General.kMaxAngularSpeedRadiansPerSecond;
      valueOfOmega /= (1
          - Constants.OI.Controller.kDeadband * Constants.SwerveDrive.General.kMaxAngularSpeedRadiansPerSecond);
      return valueOfOmega * signOfOmega;
    }

  }

  public SwerveModuleState[] GetModuleStates(double vxMetersPerSecond, double vyMetersPerSecond,
      double omegaRadiansPerSecond) {

    if (this.isFieldCentric && this.fieldCentricOverride == false) {
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond,
          omegaRadiansPerSecond, Rotation2d.fromDegrees(getGyro()));
      return Constants.SwerveDrive.Kinematics.kDriveKinematics
          .toSwerveModuleStates(chassisSpeeds);
    } else {
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond,
          omegaRadiansPerSecond, Rotation2d.fromDegrees(0));
      return Constants.SwerveDrive.Kinematics.kDriveKinematics
          .toSwerveModuleStates(chassisSpeeds);
    }
  }

  public static SwerveModuleState[] OptimizeModuleStates(
      SwerveModuleState[] swerveModuleStates,
      double[] moduleAngles) {
    swerveModuleStates[0] = SwerveModuleState.optimize(
        swerveModuleStates[0],
        Rotation2d.fromDegrees(moduleAngles[0]));
    swerveModuleStates[1] = SwerveModuleState.optimize(
        swerveModuleStates[1],
        Rotation2d.fromDegrees(moduleAngles[1]));
    swerveModuleStates[2] = SwerveModuleState.optimize(
        swerveModuleStates[2],
        Rotation2d.fromDegrees(moduleAngles[2]));
    swerveModuleStates[3] = SwerveModuleState.optimize(
        swerveModuleStates[3],
        Rotation2d.fromDegrees(moduleAngles[3]));

    return swerveModuleStates;
  }

  private class SwerveModule {
    // The SwerveModule's Turn Motor
    private final CANSparkMax turnMotor;
    // The SwerveModule's Turn Encoder
    private final WPI_CANCoder turnEncoder;
    // The SwerveModule's Drive Motor
    private final WPI_TalonFX driveMotor;
    // The PID Controller for the SwerveModule's Turn Motor
    private final PIDController turnMotorPIDController;

    private double speed = 0;
    private double currentTurnPosition = 0;

    public SwerveModule(
        double kP_Turn,
        double kI_Turn,
        double kD_Turn,
        int kTurnMotorPort,
        int kTurnEncoderPort,
        int kDriveMotorPort) {

      this.turnMotor = new CANSparkMax(kTurnMotorPort, MotorType.kBrushless);
      this.turnEncoder = new WPI_CANCoder(kTurnEncoderPort);

      this.turnMotorPIDController = new PIDController(kP_Turn, kI_Turn, kD_Turn);
      this.turnMotorPIDController.enableContinuousInput(-180, 180);

      this.driveMotor = new WPI_TalonFX(kDriveMotorPort);
      PhysicsSim.getInstance().addTalonFX(this.driveMotor, 0.75, 20660);

      this.driveMotor.setInverted(true);
      this.driveMotor.set(ControlMode.PercentOutput, 0);
      this.driveMotor.setSelectedSensorPosition(0);
      this.driveMotor.setNeutralMode(NeutralMode.Brake);
      /* Factory Default all hardware to prevent unexpected behaviour */
      this.driveMotor.configFactoryDefault();

      /* Config neutral deadband to be the smallest possible */
      this.driveMotor.configNeutralDeadband(0.001);

      /* Config sensor used for Primary PID [Velocity] */
      this.driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
          Constants.SwerveDrive.Module.PID.kPIDLoopIdx,
          Constants.SwerveDrive.Module.PID.kTimeoutMs);

      /* Config the peak and nominal outputs */
      this.driveMotor.configNominalOutputForward(0, Constants.SwerveDrive.Module.PID.kTimeoutMs);
      this.driveMotor.configNominalOutputReverse(0, Constants.SwerveDrive.Module.PID.kTimeoutMs);
      this.driveMotor.configPeakOutputForward(1, Constants.SwerveDrive.Module.PID.kTimeoutMs);
      this.driveMotor.configPeakOutputReverse(-1, Constants.SwerveDrive.Module.PID.kTimeoutMs);

      /* Config the Velocity closed loop gains in slot0 */
      this.driveMotor.config_kF(Constants.SwerveDrive.Module.PID.kPIDLoopIdx,
          Constants.SwerveDrive.Module.PID.k_Velocity_F,
          Constants.SwerveDrive.Module.PID.kTimeoutMs);
      this.driveMotor.config_kP(Constants.SwerveDrive.Module.PID.kPIDLoopIdx,
          Constants.SwerveDrive.Module.PID.k_Velocity_P,
          Constants.SwerveDrive.Module.PID.kTimeoutMs);
      this.driveMotor.config_kI(Constants.SwerveDrive.Module.PID.kPIDLoopIdx,
          Constants.SwerveDrive.Module.PID.k_Velocity_I,
          Constants.Shooter.PID.kTimeoutMs);
      this.driveMotor.config_kD(Constants.SwerveDrive.Module.PID.kPIDLoopIdx,
          Constants.SwerveDrive.Module.PID.k_Velocity_D,
          Constants.SwerveDrive.Module.PID.kTimeoutMs);
      /*
       * Talon FX does not need sensor phase set for its integrated sensor
       * This is because it will always be correct if the selected feedback device is
       * integrated sensor (default value)
       * and the user calls getSelectedSensor* to get the sensor's position/velocity.
       * 
       * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
       * sensor-phase
       */
      // _talon.setSensorPhase(true);
      this.driveMotor.set(ControlMode.PercentOutput, 0);

    }

    public SwerveModuleState getState() {
      return new SwerveModuleState(
          this.driveMotor.getSensorCollection().getIntegratedSensorVelocity()
              / Constants.SwerveDrive.Module.General.MPSToTickPerDS,
          Rotation2d.fromDegrees(this.turnEncoder.getAbsolutePosition()));

    }

    public double getModuleAngle() {
      return this.turnEncoder.getAbsolutePosition();
    }

    public void setModule(SwerveModuleState swerveModuleState) {

      Rotation2d angle = swerveModuleState.angle;
      this.speed = swerveModuleState.speedMetersPerSecond;
      this.currentTurnPosition = this.turnEncoder.getAbsolutePosition();

      double myAngle = swerveModuleState.angle.getDegrees();
      double myAngleRot = (myAngle <= 180 && myAngle >= 0) ? myAngle * 2048 / 180 : (myAngle + 360) * 4096 / 360;

      this.turnEncoder.getSimCollection().setRawPosition((int) myAngleRot);

      this.turnMotorPIDController.setSetpoint(angle.getDegrees());
      double output = this.turnMotorPIDController.calculate(currentTurnPosition);
      this.turnMotor.set(output);

      double ticksPerDS = this.speed * Constants.SwerveDrive.Module.General.MPSToTickPerDS;

      this.driveMotor.set(ControlMode.Velocity, ticksPerDS);

    }

    public void turnOffModule() {
      this.turnMotor.set(0);
      this.driveMotor.set(ControlMode.PercentOutput, 0);
    }

  }

  public void ZeroWheels() {
    SwerveModuleState zeroState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    this.frontLeftSwerveModule.setModule(zeroState);
    this.frontRightSwerveModule.setModule(zeroState);
    this.backLeftSwerveModule.setModule(zeroState);
    this.backRightSwerveModule.setModule(zeroState);
  }

  public void setFieldCentricOverride(boolean fieldCentricOverride) {
    this.fieldCentricOverride = fieldCentricOverride;
  }

}
