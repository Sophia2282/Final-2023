package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private final CANSparkMax arm_motor = new CANSparkMax(5, MotorType.kBrushless); // read parameters from Constants.java
  private RelativeEncoder relativeEncoder = arm_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  public double currentEncoderPosition = 0;

  public Arm() {
    relativeEncoder.setPositionConversionFactor(Constants.Encoder.NEO_POSITION_CONVERSION_FACTOR);
  }

  // Move
  public void raise(double speed) {
    arm_motor.set(speed);
  }

  public void lower(double speed) {
    arm_motor.set(speed);
  }

  public void armstop() {
    arm_motor.set(0);
  }

  public double getMeasurement() {
    return relativeEncoder.getPosition();
  }

  @Override
  public void periodic() {
    currentEncoderPosition = getMeasurement();
    SmartDashboard.putNumber("Arm Position (Relative Encoder)", getMeasurement());
  }
}
