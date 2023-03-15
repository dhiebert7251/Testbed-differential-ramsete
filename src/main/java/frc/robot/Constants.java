// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

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
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    //drive motor CAN addresses
    public static final int kFrontLeftMotorID = 20;
    public static final int kBackLeftMotorID = 21;
    public static final int kFrontRightMotorID = 22;
    public static final int kBackRightMotorID = 23;

    public static final boolean kFrontLeftMotorInverted = false;
    public static final boolean kBackLeftMotorInverted = false;
    public static final boolean kFrontRightMotorInverted = true;
    public static final boolean kBackRightMotorInverted = true;

    public static final int kFrontLeftEncoderA = 0;
    public static final int kFrontLeftEncoderB = 1;
    public static final int kBackLeftEncoderA = 2;
    public static final int kBackLeftEncoderB = 3;
    public static final int kFrontRightEncoderA = 4;
    public static final int kFrontRightEncoderB = 5;
    public static final int kBackRightEncoderA = 6;
    public static final int kBackRightEncoderB = 7
    ;

    public static final boolean kGyroReversed = false;

    //TODO:  adjust turn constants
    public static final double kP_Turn = 1.0;
    public static final double kI_Turn = 0.0;
    public static final double kD_Turn = 0.0;

    public static final double kP_Stabilization = 1.0;
    public static final double kI_Stabilization = 0.0;
    public static final double kD_Stabilization = 0.0;



    public static final double kTurnToleranceDeg = 3.0;  //accuracy to setpoint before command ends
    public static final double kTurnRateToleranceDegPerS = 10.0; //degrees per second
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;
    public static final double kMaxTurnRateDegPerS = 100;


  }

  public static class PhysicalConstants {

    private static final double kMotorFreeSpeed = 5330.0; //CIM motor
    private static final double kDriveGearRatio = 1/12.75; //Toughbox Micro
    private static final double kWheelDiameter = Units.inchesToMeters(8); //pneumatic wheels
    //public static final double kWheelDiameter = Units.inchesToMeters(6); //meccanum or Hi-Tec traction wheels
    public static final double kMaxSpeed = (kMotorFreeSpeed*kDriveGearRatio/60)*(Math.PI*kWheelDiameter); // meters per second
    public static final double kMaxAccel = 1.0;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
  
    public static final double kTrackWidth = Units.inchesToMeters(33.12); // from drawing
    public static final double kWheelBase = Units.inchesToMeters(19.945); // from drawing

    public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackWidth);

    //private static final double kWheelRadius = kWheelDiameter/2; // meters
    private static final int kEncoderResolution = 80; //CIMcoder (20 per channel quadrature) - TODO:is this correct?
    public static final double kDistancePerPulse = (Math.PI*kWheelDiameter)/kEncoderResolution;
  }

  public static class AutoConstants {

    public static final double kAutoDriveDistanceMeters = Units.inchesToMeters(60);
    private static final double kAutoDriveSpeedScale = 0.50;
    public static final double kAutoDriveSpeed = PhysicalConstants.kMaxSpeed * kAutoDriveSpeedScale; 

    //TODO: determine values using SYSId
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    //TODO: inital values.  Should be reset
    public static final double kRamseteB = 2.0;
    public static final double kRamseteZeta = 0.7;

    //TODO: adjust velocity PID values
    public static final double kPDriveVel = 8.5; //value seems very high
    public static final double kIDriveVel = 0.0;
    public static final double kDDriveVel = 0.0;

  }
}
