// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.Optional;

import com.choreo.lib.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {  
      
  private static Command lastCommand;
  private static Command selectedAuto;


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

  ChoreoTrajectory traj = Choreo.getTrajectory("r1_1BallAmp");


  Choreo.choreoSwerveCommand(
    traj,
    this::getPose
    new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0),
    new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), 
    new PIDController(Constants.AutoConstants.kPThetaController, 0.0, 0.0), 
    (ChassisSpeeds speeds) ->
        this.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), ...),
    () -> {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            mirror = alliance.isPresent() && alliance.get() == Alliance.Red;
    },
    this, 
);
}
