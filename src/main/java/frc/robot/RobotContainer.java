// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerPorts;
import frc.robot.Constants.Speeds;
import frc.robot.commands.ArcadeDrive;
//import frc.robot.commands.AutoBalance;
//import frc.robot.commands.BalanceCalc;
// import frc.robot.commands.DriveTime;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.IntakeObject;
import frc.robot.commands.LiftArm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private final Intake m_intake = new Intake();
        private final Arm m_arm = new Arm();
        private final Elevator m_elevator = new Elevator();
        private final DriveTrain m_drive = new DriveTrain();
        // private final BalanceCalc m_balancecalc = new BalanceCalc();

        public XboxController driverController = new XboxController(ControllerPorts.kDriverControllerPort);
        public Joystick subControllerJoystick = new Joystick(ControllerPorts.kJoystickPort);

        public RobotContainer() {     //lift arm up and dowm
                m_arm.setDefaultCommand(
                                new LiftArm(m_arm, () -> subControllerJoystick.getRawButton(6),
                                                () -> subControllerJoystick.getRawButton(4)));

                m_intake.setDefaultCommand( //intake in out and slow hold
                                new IntakeObject(m_intake, () -> subControllerJoystick.getRawButton(3),
                                                () -> subControllerJoystick.getRawButton(5),
                                                () -> subControllerJoystick.getRawButton(2)));

                m_elevator.setDefaultCommand( //elevator up and dowm
                                new ElevatorUp(m_elevator, () -> subControllerJoystick.getRawButton(10),
                                                () -> subControllerJoystick.getRawButton(8)));

                configureBindings();
                m_drive.setDefaultCommand(getarcadeDriveCommand());
        }

        private void configureBindings() {

                // Schedule `exampleMethodComfmand` when the Xbox controller's B button is
                // pressed,
                // cancelling on release.
                // driverController.getRawButton(1).whileTrue(arcadeDrive.());
        }

        public Command getAutonomousCommand() {
                // return new InstantCommand();
                return new SequentialCommandGroup(
                                new StartEndCommand(
                                                () -> m_arm.setArmSpeed(Speeds.ARM_SPEED),
                                                () -> m_arm.armstop(), m_arm).withTimeout(1.5),
                                new StartEndCommand(
                                                () -> m_elevator.setElevatorSpeed(Speeds.ELEVATOR_SPEED),
                                                () -> m_elevator.ElevatorStop(), m_elevator).withTimeout(1.2),
                                new StartEndCommand(
                                                () -> m_intake.setIntakeSpeed(Speeds.INTAKE_SPEED * -1),
                                                () -> m_intake.stopIntake(), m_intake).withTimeout(1),
                                new StartEndCommand(
                                                () -> m_arm.setArmSpeed(Speeds.ARM_SPEED * -1),
                                                () -> m_arm.armstop(), m_arm).withTimeout(1.5),
                                new StartEndCommand(
                                                () -> m_elevator.setElevatorSpeed(Speeds.ELEVATOR_SPEED * -1),
                                                () -> m_elevator.ElevatorStop(), m_elevator).withTimeout(1),
                                new StartEndCommand(
                                                () -> m_drive.setPower(Speeds.DRIVE_SPEED * -1),
                                                () 
                                                () -> m_drive.stop(), m_drive).withTimeout(2));
                // new StartEndCommand(
                // try during practice match () -> m_drive.setPower(Speeds.DRIVE_SPEED * -1),
                // () -> m_drive.stop(), m_drive).withTimeout(1.8));
                // new StartEndCommand(
                // () -> m_arm.setArmSpeed(Speeds.ARM_SPEED),
                // () -> m_arm.armstop(), m_arm).withTimeout(1.25),

        }

        // m_drive.setDefaultCommand(
        // new slowMode(m_drive,() -> driverController.getRawButton(1));

        public Command getarcadeDriveCommand() {  //slow mode
                return new ArcadeDrive(
                                m_drive,
                                () -> (-driverController.getRawAxis(1) * (driverController.getAButton() ? 0.7 : 1)), 
                                () -> (-driverController.getRawAxis(4) * (driverController.getAButton() ? 0.7 : 1)));
        }
}
