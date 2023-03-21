// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeObject extends CommandBase {
  private final Supplier<Boolean> intake, outake, hold;

  private final Intake m_intake;
  double m_speed;

  public IntakeObject(Intake subsystem, Supplier<Boolean> intakeButton, Supplier<Boolean> outakeButton,
      Supplier<Boolean> holdButton) {
    m_intake = subsystem;
    intake = intakeButton;
    outake = outakeButton;
    hold = holdButton;
    addRequirements(m_intake);
  }
  /*
   * public IntakeObject(Intake subsystem, Supplier<Boolean> intakeButton,
   * Supplier<Boolean> outakeButton) {
   * m_intake = subsystem; orginal!!!
   * intake = intakeButton;
   * outake = outakeButton;
   * addRequirements(m_intake);
   * }
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (intake.get()) {
      m_intake.intake(0.7);
    } else if (outake.get()) {
      m_intake.outake(-0.3);
    } else if (hold.get()) {
      m_intake.hold(-0.1);
    } else {
      m_intake.stopIntake();
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

