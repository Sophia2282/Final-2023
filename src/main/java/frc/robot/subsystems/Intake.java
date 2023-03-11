// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  CANSparkMax LeftIntake = new CANSparkMax(8, MotorType.kBrushless);
  CANSparkMax RightIntake = new CANSparkMax(9, MotorType.kBrushless);

  public Intake() {
  }

  public void setIntakeSpeed(double speed) {
    LeftIntake.set(speed);
    RightIntake.set(-speed);
  }

  public void outake(double speed) {
    LeftIntake.set(-speed);
    RightIntake.set(speed);
  }

  public void hold(double speed) {
    LeftIntake.set(speed);
    RightIntake.set(-speed);
  }

  public void stopIntake() {
    LeftIntake.set(0);
    RightIntake.set(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
