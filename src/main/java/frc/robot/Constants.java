// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
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
  
  public static final class SwerveConstants {

    public static final int pigeonID = 15;

    public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
    .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = 0.5715;
    public static final double wheelBase = 0.5715;
    public static final double wheelCircumference = chosenModule.wheelCircumference;
    /*
    * Swerve Kinematics
    * No need to ever change this unless you are not doing a traditional
    * rectangular/square 4 module swerve
    */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    );
    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limitting */ // DID NOT CHANGE
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35; 
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* Max Voltages - subject to change */
    public static final int driveMaxVoltage = 30;
    public static final int angleMaxVoltage = 30;

    /*
      * These values are used by the drive falcon to ramp in open loop and closed
      * loop driving.
      * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
    */ // DID NOT CHANGE
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /*
     * Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE
     */
    public static final double driveKS = (0.15932 / 12); 
    public static final double driveKV = (2.3349 / 12); 
    public static final double driveKA = (0.47337 / 12);

    /* Swerve Profiling Values */
    /* Meters per Second */
    public static final double maxSpeed = 4.5; 
    /** Radians per Second */
    public static final double maxAngularVelocity = 7.0; 

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

 
    /* Module Specific Constants - TO BE DONE FOR ARTEMIS*/ 
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
        public static final int driveMotorID = 2;
        public static final int angleMotorID = 1;
        public static final int canCoderID = 9;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(338.17291259765625);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
        public static final int driveMotorID = 8;
        public static final int angleMotorID = 7;
        public static final int canCoderID = 12;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(165.399169921875);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
        public static final int driveMotorID = 4;
        public static final int angleMotorID = 3;
        public static final int canCoderID = 10;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(350.9994506835938);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
        public static final int driveMotorID = 6;
        public static final int angleMotorID = 5;
        public static final int canCoderID = 11;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(33.3597412109375);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                canCoderID, angleOffset);
    }
  }
}
