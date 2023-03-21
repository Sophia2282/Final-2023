// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerPorts;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.BalanceCalc;
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
  private final BalanceCalc m_balancecalc = new BalanceCalc();

  public XboxController driverController = new XboxController(ControllerPorts.kDriverControllerPort);
  public Joystick subControllerJoystick = new Joystick(ControllerPorts.kJoystickPort);

  public RobotContainer() {
    m_arm.setDefaultCommand(
        new LiftArm(m_arm, () -> subControllerJoystick.getRawButton(6),
            () -> subControllerJoystick.getRawButton(4)));

    m_intake.setDefaultCommand(
        new IntakeObject(m_intake, () -> subControllerJoystick.getRawButton(3),
            () -> subControllerJoystick.getRawButton(5), () -> subControllerJoystick.getRawButton(2)));

    // m_elevator.setDefaultCommand(
    // new ElevatorUp(m_elevator, () -> subControllerJoystick.getRawAxis(1) * 0.5));

    m_elevator.setDefaultCommand(
        new ElevatorUp(m_elevator, () -> subControllerJoystick.getRawButton(10),
            () -> subControllerJoystick.getRawButton(8)));

    configureBindings();
    m_drive.setDefaultCommand(getarcadeDriveCommand());

    // m_arm.setDefaultCommand(new LiftArm(m_arm, 0));

    // m_intake.setDefaultCommand(new IntakeObject(m_intake, 0));
    // m_elevator.setDefaultCommand(new ElevatorUp(m_elevator, 0));
  }

  private void configureBindings() {

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // driverController.getRawButton(1).whileTrue(arcadeDrive.());
  }

  public Command getAutonomousCommand() {
    // return new InstantCommand();
    return new SequentialCommandGroup(
      new AutoBalance(m_balancecalc)
    );

    
    // new DriveTime(Speeds.DRIVE_SPEED, 3, m_drive)
    // new DriveTrain(Speeds.DRIVE_SPEED, 2, m_drive);
    // new LiftArm(m_arm, Speeds.ARM_SPEED).withTimeout(1),
    // new DriveTrain(Speeds.DRIVE_SPEED, 2, m_drive),
    // new LiftArm(m_arm, Speeds.ARM_SPEED).withTimeout(0.5),
    // new ElevatorUp(m_elevator, Speeds.ELEVATOR_SPEED).withTimeout(0.7),
    // new IntakeObject(m_intake, -0.80).withTimeout(1),
    // new DriveTime(Speeds.DRIVE_SPEED, 2, m_drive));
    // new LiftArm(m_arm, Speeds.ARM_SPEED).withTimeout(1);
  }

  // m_drive.setDefaultCommand(
  // new slowMode(m_drive,() -> driverController.getRawButton(1));

  public Command getarcadeDriveCommand() {
    return new ArcadeDrive(
        m_drive, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(4),
        () -> driverController.getRawButton(1)); // change into left bumper?
  }
}
