// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LiftArm extends CommandBase {
  private final Supplier<Boolean> up, down;

  private final Arm m_arm;
  double m_speed;

  public LiftArm(Arm subsystem, Supplier<Boolean> upButton, Supplier<Boolean> downButton) {
    m_arm = subsystem;
    up = upButton;
    down = downButton;
    addRequirements(m_arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (up.get() && m_arm.currentEncoderPosition > Constants.Encoder.ARM_HIGHEST_POSITION) {
      m_arm.raise(-1.0);
    } else if (down.get() && m_arm.currentEncoderPosition < Constants.Encoder.ARM_LOWEST_POSITION) {
      m_arm.lower(1.0);
    } else {
      m_arm.armstop();
    }

  }

  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
