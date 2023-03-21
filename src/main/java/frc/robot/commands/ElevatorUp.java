// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;

public class ElevatorUp extends CommandBase {
  private final Elevator m_elevator;
  private final Supplier<Boolean> elevatorUp;
  private final Supplier<Boolean> elevatorDown;
  double m_speed;

  public ElevatorUp(Elevator subsystem, Supplier<Boolean> elevatorDownButton, Supplier<Boolean> elevatorUpButton) {
    m_elevator = subsystem;
    elevatorDown = elevatorDownButton;
    elevatorUp = elevatorUpButton;
    addRequirements(m_elevator);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevatorUp.get()){ // && m_elevator.currentEncoderPosition > Constants.Encoder.ELEVATOR_HIGHEST_POSITION) { 
      m_elevator.ElevatorUp(0.75);
    } else if (elevatorDown.get() && m_elevator.currentEncoderPosition < Constants.Encoder.ELEVATOR_LOWEST_POSITION) {
      m_elevator.ElevatorDown(0.65);
    } else {
      m_elevator.ElevatorStop();
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
