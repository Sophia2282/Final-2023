// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenixpro.signals.InvertedValue;

//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.commands.tankDrive;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenixpro.signals.InvertedValue;
//import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.commands.tankDrive;
//import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
  WPI_TalonSRX leftA;
  WPI_TalonSRX leftB;
  WPI_TalonSRX rightA;
  WPI_TalonSRX rightB;
  MotorControllerGroup leftMotors;
  MotorControllerGroup rightMotors;
  DifferentialDrive diffDrive;

  // WPI_TalonSRX leftA = new WPI_TalonSRX(Constants.MotorPorts.LEFT_MOTOR);
  // WPI_TalonSRX leftB = new WPI_TalonSRX(Constants.MotorPorts.LEFT_MOTORA);
  // WPI_TalonSRX rightA = new WPI_TalonSRX(Constants.MotorPorts.RIGHT_MOTOR);
  // WPI_TalonSRX rightB = new WPI_TalonSRX(Constants.MotorPorts.RIGHT_MOTORA);

  // MotorControllerGroup leftMotors = new MotorControllerGroup(leftA, leftB);
  // MotorControllerGroup rightMotors = new MotorControllerGroup(rightA, rightB);
  // DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);

  public DriveTrain() {
    System.out.println("drivetrain created");
    leftA = new WPI_TalonSRX(Constants.MotorPorts.LEFT_MOTOR);
    leftB = new WPI_TalonSRX(Constants.MotorPorts.LEFT_MOTORA);
    rightA = new WPI_TalonSRX(Constants.MotorPorts.RIGHT_MOTOR);
    rightB = new WPI_TalonSRX(Constants.MotorPorts.RIGHT_MOTORA);
    leftMotors = new MotorControllerGroup(leftA, leftB);
    rightMotors = new MotorControllerGroup(rightA, rightB);
    diffDrive = new DifferentialDrive(leftMotors, rightMotors);
    diffDrive.setMaxOutput(0.67);
    rightMotors.setInverted(true);
    diffDrive.setSafetyEnabled(false);
  }

  public void arcadeDrive(double forward, double turn) {
    diffDrive.arcadeDrive(forward, turn);
  }

  public void slowMode(double forward, double turn) {
    diffDrive.arcadeDrive(forward * 0.5, turn * 0.5);
  }

  // for autobalance
  public void setSpeed(double speed) {
    rightMotors.set(speed);
    leftMotors.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
