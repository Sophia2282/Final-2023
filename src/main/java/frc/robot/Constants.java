// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class ControllerPorts {
    public static final int kDriverControllerPort = 0;
    public static final int kJoystickPort = 1;

  }

  public static class MotorPorts {

    public static final int LEFT_MOTOR = 1;
    public static final int LEFT_MOTORA = 3;
    public static final int RIGHT_MOTOR = 2;
    public static final int RIGHT_MOTORA = 4;
    public static final int ARMZ = 5;
    public static final int ELE1 = 6;
    public static final int ELE2 = 7;
    public static final int INTAKEA = 8;
    public static final int INTAKEB = 9;

  }

  public static class Speeds {

    public static final double ARM_SPEED = 0.6;
    public static final double DRIVE_SPEED = 0.6;
    public static final double INTAKE_SPEED = 0.4;

  }

  public static class Encoder {
    public static final double NEO_POSITION_CONVERSION_FACTOR = 20.1 / 360; 
    public static final double ARM_LOWEST_POSITION = 15;
    public static final double ARM_HIGHEST_POSITION = -3;
    public static final double ELEVATOR_LOWEST_POSITION = 0; // TODO: fix it
    public static final double ELEVATOR_HIGHEST_POSITION = 0; // TODO: fix it
  }
}
