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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public final class Autos {  
      
  private static Command lastCommand;
  private static Command selectedAuto;
  ChoreoTrajectory traj;
  private static Swerve s_Swerve = Swerve.getInstance();


  // Return auto selected in Shuffleboard
  public static Command getAutoCommand(AutoType auto) {
      switch (auto) {
      case Test:
          return selectedAuto = test();
      case Test2:
          return selectedAuto = test2();
      case Test3:
          return selectedAuto = test3();
      default:
          break;
      }
      return null;
  }

  public enum AutoType {
      Test,
      Test2,
      Test3
  }

  public static Command getSelectedAuto() {
    return selectedAuto;
  }

  public static void cancelLastCommand() {
    lastCommand.cancel();
  }


  private static Command test() {
    return null;
  }
  
  private static Command test2() {
    return null;
  }

  private static Command test3() {
    return null;
  }

  private Autos() {
      throw new UnsupportedOperationException("This is a utility class!");
  }

  public Command oneBallAmp() {
  ChoreoTrajectory oneBallAmp = Choreo.getTrajectory("r1_1BallAmp");
    PIDController thetaController = new PIDController(0, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    s_Swerve.resetOdometry(oneBallAmp.getInitialPose());
    Command swerveCommand = Choreo.choreoSwerveCommand(
      oneBallAmp, 
      s_Swerve::getPose, 
      new PIDController(0, 0, 0), //TODO write actual PID values
      new PIDController(0, 0, 0),
      thetaController,
      (ChassisSpeeds speeds) -> s_Swerve.autoDrive(speeds, false), //this has to be robot-relative, need to check that auto-drive function works for this (may have to use drive function and set field-relative to false idk)
      true, //decides whether or not the math should be mirrored (depends on alliance)
      s_Swerve);
  }
}
