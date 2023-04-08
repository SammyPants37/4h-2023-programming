// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // ports for controllers
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int CO_DRIVER_CONTROLLER_PORT = 1;
  }

  public static class driveTrainConstants {
    // ports for motors
    public static final int frontLeftPort = 1;
    public static final int frontRightPort = 2;
    public static final int backLeftPort = 3;
    public static final int backRightPort = 4;

    // balancing pid
    public final static double k_balanceP = 0.01215;
    public final static double k_balanceI = 0;
    public final static double k_balanceD = 0;

    // driving pid
    public final static double k_driveP = 0;
    public final static double k_driveI = 0;
    public final static double k_driveD = 0;

    // slew rate constant
    public final static double driveStartSlewRate = 2.25;
    public final static double driveStopSlewRate = -4.5;

    // gear reduction math
    public final static int SENSOR_UNITS_PER_ROTATION = 2048;
    private final static double GEAR_REDUCTION = 5.51;
    private final static double WHEEL_RADIUS = 3;
    private final static double RADS_2_INCHES = 2 * Math.PI * WHEEL_RADIUS;
    public final static double feet2tick = (GEAR_REDUCTION) * (SENSOR_UNITS_PER_ROTATION) / (RADS_2_INCHES / 12);
    public final static double tick2feet = 1/feet2tick;
  }

  public static class ElevatorConstants{
    // motor ports
    public static final int LeftElevatorMotor = 6;
    public static final int RightElevatorMotor = 7;

    // pid
    public final static double kp = 0;
    public final static double ki = 0;
    public final static double kd = 0;

    // gear ratio math
    private final static double GEAR_REDUCTION = 20;
    private final static double WINCH_RADIUS = 1.72/2;
  }

  public static class ArmMotorConstants{
    // motor port
    public static final int armMotor = 5;

    // pid
    public final static double kp = 0;
    public final static double ki = 0;
    public final static double kd = 0;

    // gear ratio math
    private final static double GEAR_REDUCTION = 20;
    private final static double WINCH_RADIUS = 1.9/2;
  }

  public static class endEffectorConstants {
    // motor ports
    public static final int leftMotor = 8;
    public static final int rightMotor = 9;

    // pid
    public final static double kp = 0;
    public final static double ki = 0;
    public final static double kd = 0;

    // gear ratio math
    private final static double GEAR_REDUCTION = 25;
    private final static double WINCH_RADIUS = 1.29/2;

    public final static double kCircumperence = 2 * Math.PI * WINCH_RADIUS;
    public final static double kInches2Rots = kCircumperence / GEAR_REDUCTION;
    public final static double kRots2inches = 1/kInches2Rots;

    public final static double doubleSoftLimitRots = 20;
    public final static float rightSoftLimitRots = (float) doubleSoftLimitRots;
    public final static float leftSoftLimitRots = (float) doubleSoftLimitRots;
  }

  public static final class IOConstants {
    // === XBOX CHANNELS === //
    // AXES
    public static final int leftXAxisChannel = 0;
    public static final int leftYAxisChannel = 1;
    public static final int leftTriggerChannel = 2;
    public static final int rightTriggerChannel = 3;
    public static final int rightXAxisChannel = 4;
    public static final int rightYAxisChannel = 5;

    // BUTTONS
    public static final int aButtonChannel = 1;
    public static final int bButtonChannel = 2;
    public static final int xButtonChannel = 3;
    public static final int yButtonChannel = 4;

    public static final int leftBumperChannel = 5;
    public static final int rightBumperChannel = 6;

    public static final int backButtonChannel = 7;
    public static final int startButtonChannel = 8;

    public static final int POVU = 0;
    public static final int POVR = 90;
    public static final int POVD = 180;
    public static final int POVL = 270;
  }
}