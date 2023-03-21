// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  CANSparkMax elevator_motorA = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax elevator_motorB = new CANSparkMax(7, MotorType.kBrushless);
  private RelativeEncoder relativeEncoderA = elevator_motorA.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  private RelativeEncoder relativeEncoderB = elevator_motorB.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  public double currentEncoderPosition = 0;

  public Elevator() {
    relativeEncoderA.setPositionConversionFactor(Constants.Encoder.NEO_POSITION_CONVERSION_FACTOR);
    relativeEncoderB.setPositionConversionFactor(Constants.Encoder.NEO_POSITION_CONVERSION_FACTOR);
  }

  public void setElevatorSpeed(double speed) {
    elevator_motorA.set(speed);
    elevator_motorB.set(-speed);
  }

  public void ElevatorStop() {
    elevator_motorA.set(0);
    elevator_motorB.set(0);
  }

  public void ElevatorUp(double speed) {
    elevator_motorA.set(speed);
    elevator_motorB.set(-speed);
  }

  public void ElevatorDown(double speed) {
    elevator_motorA.set(-speed);
    elevator_motorB.set(speed);
  }

  public double getMeasurement(RelativeEncoder relativeEncoder) {
    return relativeEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder Position A", getMeasurement(relativeEncoderA));
    SmartDashboard.putNumber("Elevator Encoder Position B", getMeasurement(relativeEncoderB));
    currentEncoderPosition = getMeasurement(relativeEncoderA);
  }
}
