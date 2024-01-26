// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.Optional;

import com.choreo.lib.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public final class Autos {  
      
  private static Command autoCommand;
  private static Command selectedAuto;
  ChoreoTrajectory traj;
  private static Swerve s_Swerve = Swerve.getInstance();


  // Return auto selected in Shuffleboard
  public static Command getAutoCommand(AutoType auto) {
      switch (auto) {
      case Test:
          return selectedAuto = test();
      case oneBallAmp:
          return selectedAuto = oneBallAmp();
      case ballSpeaker:
          return selectedAuto = ballSpeaker();
      default:
          break;
      }
      return null;
  }

  public enum AutoType {
      Test,
      oneBallAmp,
      ballSpeaker
  }

  public static Command getSelectedAuto() {
    return selectedAuto;
  }

  public static void cancelAutoCommand() {
    autoCommand.cancel();
  }


  private static Command test() {
    SmartDashboard.putBoolean("test auto successful", true);
    return null;
  }
  
  public static Command oneBallAmp() {
    
    ChoreoTrajectory oneBallAmp = Choreo.getTrajectory("r1_1BallAmp");
    
    // TODO write real pid values looool
    PIDController xController = new PIDController(0, 0, 0);
    PIDController yController = new PIDController(0, 0, 0);
    PIDController thetaController = new PIDController(0, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    s_Swerve.resetOdometry(oneBallAmp.getInitialPose());
    autoCommand = Choreo.choreoSwerveCommand(
      oneBallAmp, 
      s_Swerve::getPose, 
      xController,
      yController,
      thetaController,
      (ChassisSpeeds speeds) -> s_Swerve.autoDrive(speeds, false), //this has to be robot-relative, need to check that auto-drive function works for this (may have to use drive function and set field-relative to false idk)
      () -> { return true; }, //decides whether or not the math should be mirrored (depends on alliance)
      s_Swerve);

      return autoCommand;
  }

  public static Command ballSpeaker() {
    
    ChoreoTrajectory ballSpeaker = Choreo.getTrajectory("r1_1BallSpeaker");
    
    PIDController xController = new PIDController(0, 0, 0);
    PIDController yController = new PIDController(0, 0, 0);
    PIDController thetaController = new PIDController(0, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    s_Swerve.resetOdometry(ballSpeaker.getInitialPose());
    autoCommand = Choreo.choreoSwerveCommand(
      ballSpeaker, 
      s_Swerve::getPose, 
      xController,
      yController,
      thetaController,
      (ChassisSpeeds speeds) -> s_Swerve.autoDrive(speeds, false), //this has to be robot-relative, need to check that auto-drive function works for this (may have to use drive function and set field-relative to false idk)
      () -> { return true; }, //decides whether or not the math should be mirrored (depends on alliance)
      s_Swerve);

      return autoCommand;
  }


  private Autos() {
      throw new UnsupportedOperationException("This is a utility class!");
  }
}
