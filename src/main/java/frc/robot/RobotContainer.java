// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.Cliff;
import frc.robot.autos.GabesDadsAuto;
import frc.robot.autos.GabesDadsAutoWithLimeLight;
import frc.robot.autos.Gawrsh;
import frc.robot.autos.GawrshWithLimeLight;
import frc.robot.autos.MachoGrande;
import frc.robot.autos.MachoGrandeWithLimeLight;
import frc.robot.autos.Mammoth;
import frc.robot.autos.NewOldFaithful;
import frc.robot.autos.Norm;
import frc.robot.autos.OldFaithful;
import frc.robot.autos.Rosie;
import frc.robot.autos.Runaway;
import frc.robot.autos.RunawayWithLimeLight;
import frc.robot.autos.Sawmill;
import frc.robot.autos.ShirleyWithLimeLight;
import frc.robot.autos.sequences.AimAndShoot;
import frc.robot.commands.DriveManual;
import frc.robot.commands.Eject;
import frc.robot.commands.HighShot;
import frc.robot.commands.Load;
import frc.robot.commands.LowShot;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PincerSubsystem;
import frc.robot.subsystems.PoweredHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SingulatorSubsystem;
import frc.robot.subsystems.SwingSubSystem;
import frc.robot.subsystems.WinchSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private final DriveSubsystem driveSubsystem;
        private final ShooterSubsystem shooterSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private final IntakeArmSubsystem intakeArmSubsystem;
        private final LifterSubsystem lifterSubsystem;
        private final SingulatorSubsystem singulatorSubsystem;
        private final WinchSubsystem winchSubsystem;
        private final PincerSubsystem pincerSubsystem;
        private final CameraSubsystem cameraSubsystem;
        private final LimelightSubsystem limelightSubsystem;
        private final SwingSubSystem swingSubSystem;
        private final PoweredHoodSubsystem poweredHoodSubsystem;
        // The driver's controls
        private final XboxController driverControls;
        // The operator's controls
        private final XboxController operatorControls;
        SendableChooser<Command> m_chooser = new SendableChooser<>();

        // private final SequentialCommandGroup complexAuto0;
        // private final SequentialCommandGroup complexAuto1;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                driveSubsystem = new DriveSubsystem();
                shooterSubsystem = new ShooterSubsystem();
                lifterSubsystem = new LifterSubsystem();
                intakeSubsystem = new IntakeSubsystem();
                singulatorSubsystem = new SingulatorSubsystem();
                winchSubsystem = new WinchSubsystem();
                intakeArmSubsystem = new IntakeArmSubsystem();
                pincerSubsystem = new PincerSubsystem();
                cameraSubsystem = new CameraSubsystem();
                limelightSubsystem = new LimelightSubsystem();
                swingSubSystem = new SwingSubSystem();
                poweredHoodSubsystem = new PoweredHoodSubsystem();

                // Manual Controls
                driverControls = new XboxController(Constants.OI.Controller.kDriverPort);
                operatorControls = new XboxController(Constants.OI.Controller.kOperatorPort);

                // Default Commands
                driveSubsystem.setDefaultCommand(new DriveManual(driveSubsystem, driverControls));
                shooterSubsystem.setDefaultCommand(new RunCommand(() -> shooterSubsystem.setDefaultSpeed(), shooterSubsystem));
                winchSubsystem.setDefaultCommand(
                                new RunCommand(() -> winchSubsystem.Winch(operatorControls), winchSubsystem));
                singulatorSubsystem.setDefaultCommand(
                                new RunCommand(() -> singulatorSubsystem.Off(), singulatorSubsystem));
                intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.Off(), intakeSubsystem));
                lifterSubsystem.setDefaultCommand(new RunCommand(() -> lifterSubsystem.Off(), lifterSubsystem));
                poweredHoodSubsystem.setDefaultCommand(
                                new RunCommand(() -> poweredHoodSubsystem.Off(), poweredHoodSubsystem));

                // Autonomous Commands
                Norm norm = new Norm(driveSubsystem, intakeSubsystem, lifterSubsystem, singulatorSubsystem,
                                shooterSubsystem, pincerSubsystem, poweredHoodSubsystem);

                OldFaithful oldFaithful = new OldFaithful(driveSubsystem, intakeSubsystem, lifterSubsystem,
                                singulatorSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem);

                Sawmill sawmill = new Sawmill(driveSubsystem, intakeSubsystem, lifterSubsystem,
                                singulatorSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem);

                Mammoth mammoth = new Mammoth(driveSubsystem, intakeSubsystem, lifterSubsystem,
                                singulatorSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem);

                Rosie rosie = new Rosie(driveSubsystem, intakeSubsystem, lifterSubsystem, singulatorSubsystem,
                                shooterSubsystem, pincerSubsystem, poweredHoodSubsystem);

                GabesDadsAuto gabesDadsAuto = new GabesDadsAuto(driveSubsystem, intakeSubsystem, lifterSubsystem, singulatorSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem);
                GabesDadsAutoWithLimeLight gabesDadsAutoWithLimeLight = new GabesDadsAutoWithLimeLight(driveSubsystem, intakeSubsystem, lifterSubsystem, singulatorSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem, limelightSubsystem);
                
                Gawrsh gawrsh = new Gawrsh(driveSubsystem, intakeSubsystem, lifterSubsystem, singulatorSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem);
                GawrshWithLimeLight gawrshWithLimeLight = new GawrshWithLimeLight(driveSubsystem, intakeSubsystem, lifterSubsystem, singulatorSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem, limelightSubsystem);

                MachoGrande machoGrande = new MachoGrande(driveSubsystem, intakeSubsystem, lifterSubsystem, singulatorSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem);
                MachoGrandeWithLimeLight machoGrandeWithLimeLight2 = new MachoGrandeWithLimeLight(driveSubsystem, intakeSubsystem, lifterSubsystem, singulatorSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem, limelightSubsystem);
                Runaway runaway = new Runaway(driveSubsystem, intakeSubsystem, lifterSubsystem, singulatorSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem);
                RunawayWithLimeLight runawayWithLimeLight = new RunawayWithLimeLight(driveSubsystem, intakeSubsystem, lifterSubsystem, singulatorSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem, limelightSubsystem);

                ShirleyWithLimeLight shirleyWithLimeLight = new ShirleyWithLimeLight(driveSubsystem, intakeSubsystem, lifterSubsystem, singulatorSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem, limelightSubsystem);

                Cliff cliff = new Cliff(driveSubsystem, intakeSubsystem, lifterSubsystem, singulatorSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem, limelightSubsystem);

                NewOldFaithful newOldFaithful = new NewOldFaithful(driveSubsystem, intakeSubsystem, lifterSubsystem, singulatorSubsystem, shooterSubsystem, pincerSubsystem, poweredHoodSubsystem, intakeArmSubsystem);
                
                // Autonomous Commands to Chooser
                m_chooser.addOption("Old Faithful", oldFaithful);
                m_chooser.addOption("Sawmill", sawmill);
                m_chooser.addOption("Mammoth", mammoth);
                m_chooser.addOption("Rosie", rosie);
                m_chooser.addOption("Macho Grande", machoGrande);
                //m_chooser.addOption("Macho Grande LimeLight", machoGrandeWithLimeLight);
                m_chooser.addOption("Macho Grande LimeLight", machoGrandeWithLimeLight2);
                m_chooser.addOption("Gabe's Dad's Auto", gabesDadsAuto);
                m_chooser.addOption("Gabe's Dad's Auto Limelight", gabesDadsAutoWithLimeLight);
                m_chooser.addOption("Gawrsh", gawrsh);
                m_chooser.addOption("Gawrsh LimeLight", gawrshWithLimeLight);
                m_chooser.addOption("Runaway", runaway);
                m_chooser.addOption("Runaway Limelight", runawayWithLimeLight);
                m_chooser.addOption("Shirley Limelight", shirleyWithLimeLight);
                m_chooser.addOption("Norm", norm);
                m_chooser.addOption("Cliff", cliff);
                m_chooser.addOption("NewOldFaithful", newOldFaithful);
                


                SmartDashboard.putData("AutoChooser", m_chooser);

                // Configure the button bindings
                configureButtonBindings();

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {

                new JoystickButton(
                                driverControls,
                                Constants.OI.Buttons.X)
                                                .whenPressed(new InstantCommand(pincerSubsystem::Change,
                                                                pincerSubsystem));

                new JoystickButton(
                                driverControls,
                                Constants.OI.Buttons.BACK)
                                                .whenHeld(new RunCommand(driveSubsystem::ZeroWheels,
                                                                driveSubsystem));

                new JoystickButton(
                                driverControls,
                                Constants.OI.Buttons.START)
                                                .whenHeld(new InstantCommand(
                                                                () -> driveSubsystem.setAbsoluteOdometry(
                                                                                new Pose2d(7.651, 1.821,
                                                                                                Rotation2d.fromDegrees(
                                                                                                                0))),
                                                                driveSubsystem));

                new JoystickButton(
                                driverControls,
                                Constants.OI.Buttons.LB)
                                                .whenHeld(
                                                new AimAndShoot(driveSubsystem, limelightSubsystem,
                                                singulatorSubsystem, lifterSubsystem, intakeSubsystem,
                                                shooterSubsystem, pincerSubsystem, poweredHoodSubsystem));        
                                                

                new JoystickButton(
                                operatorControls,
                                Constants.OI.Buttons.A)
                                                .whenPressed(new InstantCommand(intakeArmSubsystem::Change,
                                                                intakeArmSubsystem));

                new JoystickButton(
                                operatorControls,
                                Constants.OI.Buttons.B)
                                                .whenPressed(new InstantCommand(swingSubSystem::Change,
                                                                swingSubSystem));

                new JoystickButton(
                                operatorControls,
                                Constants.OI.Buttons.LB)
                                                .whenHeld(new Load(lifterSubsystem, intakeSubsystem,
                                                                singulatorSubsystem));

                new JoystickButton(
                                operatorControls,
                                Constants.OI.Buttons.BACK)
                                                .whenHeld(new Eject(lifterSubsystem, intakeSubsystem,
                                                                singulatorSubsystem, shooterSubsystem, poweredHoodSubsystem));

                new JoystickButton(
                                operatorControls,
                                Constants.OI.Buttons.RB)
                                                .whenHeld(new HighShot(shooterSubsystem, singulatorSubsystem,
                                                                lifterSubsystem, intakeSubsystem, pincerSubsystem, poweredHoodSubsystem));

                new JoystickButton(
                                operatorControls,
                                Constants.OI.Buttons.Y)
                                                .whenHeld(new LowShot(shooterSubsystem, singulatorSubsystem,
                                                                lifterSubsystem, intakeSubsystem, pincerSubsystem, poweredHoodSubsystem));


                new JoystickButton(
                                operatorControls,
                                Constants.OI.Buttons.START)
                                                .whenPressed(new InstantCommand(limelightSubsystem::ToggleCameraOnOff,
                                                limelightSubsystem));

        }

        // Robot Commands on Startup and Mode Changing
        public void IntakeArmUp() {
                intakeArmSubsystem.ArmUp();
        }

        public void IntakeArmDown() {
                intakeArmSubsystem.ArmDown();
        }

        public void ShooterOff() {
                shooterSubsystem.Off();
        }

        public void LifterOff() {
                lifterSubsystem.Off();
        }

        public void IntakeOff() {
                intakeSubsystem.Off();
        }

        public void SingulatorOff() {
                singulatorSubsystem.Off();
        }

        public void ZeroWheel() {
                driveSubsystem.ZeroWheels();
        }

        public void SetFieldCentric(boolean state) {
                driveSubsystem.setFieldCentric(state);
        }

        public void StartCamera() {
                cameraSubsystem.run();
        }

        public void setAbsoluteOdometry(Pose2d pose) {
                this.driveSubsystem.setAbsoluteOdometry(pose);
        }

        public void SwingUp() {
                this.swingSubSystem.Up();
        }

        public Command getAutonomousCommand() {
                return m_chooser.getSelected();
        }

}
