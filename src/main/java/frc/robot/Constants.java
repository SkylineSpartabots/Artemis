// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  public static final int timeOutMs = 10;
  public static final double stickDeadband = 0.15;
  public static final double triggerDeadzone = 0.2;

  // hardware ports for all hardware components on the robot
  // these include CAN IDs, pneumatic hub ports, etc. 

  public static final class HardwarePorts {
    // motors (predicted) IDs not fixed
    public static final int shooterLeaderMotor = 1;
    public static final int shooterFollowerMotor = 2;
    public static final int climbLeaderMotor = 3;
    public static final int climbFollowerMotor = 4;

  }

  public static final double FIELD_WIDTH_METERS = 8.21055;
  public static final double FIELD_LENGTH_METERS = 16.54175;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class SwerveConstants {

    public static final int pigeonID = 6;

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
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Gear Ratio */
    public static final double angleEncoderGearRatio = 1;

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
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

 
    /* Module Specific Constants - TO BE DONE FOR ARTEMIS*/ 
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
        public static final int driveMotorID = 51;
        public static final int angleMotorID = 52;
        public static final int canCoderID = 9;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(338.17291259765625);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
        public static final int driveMotorID = 53;
        public static final int angleMotorID = 54;
        public static final int canCoderID = 12;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(165.399169921875);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
        public static final int driveMotorID = 55;
        public static final int angleMotorID = 56;
        public static final int canCoderID = 10;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(350.9994506835938);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
        public static final int driveMotorID = 57;
        public static final int angleMotorID = 58;
        public static final int canCoderID = 11;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(33.3597412109375);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                canCoderID, angleOffset);
    }
  }

}
