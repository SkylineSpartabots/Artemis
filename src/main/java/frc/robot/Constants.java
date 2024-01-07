// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  
  //Dummy values, need to find/calculate
  public static final List<Translation2d> kReferenceTranslations = List.of(
              new Translation2d(1, 0),
              new Translation2d(3, 3)
      );
      
  public static final class Ports{
    public static final int MASTER_SHOOTER_MOTOR = 0;
    public static final int FOLLOW_SHOOTER_MOTOR = 0; 

    public static final int FRONT_LEFT_DRIVE = 2;
    public static final int FRONT_LEFT_STEER = 1;
    public static final int FRONT_LEFT_STEER_ENCODER = 9;
    // public static final double FRONT_LEFT_OFFSET = -Math.toRadians(81.3812255859375);
    public static final double FRONT_LEFT_OFFSET = 0;

    public static final int FRONT_RIGHT_DRIVE = 8;
    public static final int FRONT_RIGHT_STEER = 7;
    public static final int FRONT_RIGHT_STEER_ENCODER = 12;
    // public static final double FRONT_RIGHT_OFFSET = -Math.toRadians(303.48358154296875);
    public static final double FRONT_RIGHT_OFFSET = 0;


    public static final int BACK_LEFT_DRIVE = 4;
    public static final int BACK_LEFT_STEER = 3;
    public static final int BACK_LEFT_STEER_ENCODER = 10;
    // public static final double BACK_LEFT_OFFSET = -Math.toRadians(349.26910400390625);
    public static final double BACK_LEFT_OFFSET = 0;


    public static final int BACK_RIGHT_DRIVE = 6;
    public static final int BACK_RIGHT_STEER = 5;
    public static final int BACK_RIGHT_STEER_ENCODER = 11;
    // public static final double BACK_RIGHT_OFFSET = -Math.toRadians(156.5277099609375);
    public static final double BACK_RIGHT_OFFSET = 0;

  }
  public static final double kTimeOutMs = 0.05; // This is a double now in seconds rather than an integer in milliseconds.
  
  public static final class DriveConstants {
      public static final double kTrackWidth = 0.4953;
      // Distance between centers of right and left wheels on robot
      public static final double kWheelBase = 0.4953;
      // Distance between front and back wheels on robot
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

      public static final boolean kGyroReversed = false;
      //Calculated via SysId
      public static final double ksVolts = 0.74397; //before 3/1: 70541, 33259, 016433
      public static final double kvVoltSecondsPerMeter = 0.33778;
      public static final double kaVoltSecondsSquaredPerMeter = 0.016934;
      //Tuned to taste for desired max velocity
      public static final double kVelocityGain = 6;
      // The maximum voltage that will be delivered to the drive motors.
      // This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
      public static final double kMaxVoltage = 12.0;
      // The maximum velocity of the robot in meters per second.
      // This is a measure of how fast the robot should be able to drive in a straight line.
      public static final double kMaxSpeedMetersPerSecond =  
      6380.0 / 60.0 *
              SdsModuleConfigurations.MK4_L2.getDriveReduction() *
              SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
      // need measure on robot
      public static final double kMaxAccelerationMetersPerSecondSquared = 10; 
      //The maximum angular velocity of the robot in radians per second.
      //This is a measure of how fast the robot can rotate in place.
      // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
      public static final double kMaxAngularSpeedRadiansPerSecond = kMaxSpeedMetersPerSecond /
            Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
      
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = kMaxAngularSpeedRadiansPerSecond;

      public static final double kpRotation = 0.1;
      public static final double kiRotation = 0.0;
      public static final double kdRotation = 0;
      public static final TrapezoidProfile.Constraints kRotationConstraints =
              new TrapezoidProfile.Constraints(
                      kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
