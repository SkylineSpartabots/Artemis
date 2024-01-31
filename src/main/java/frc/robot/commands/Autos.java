// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.ArrayList;
import java.util.Optional;

import com.choreo.lib.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Pivot.PivotState;

public final class Autos {  
      
  ChoreoTrajectory traj;
  private static Swerve s_Swerve = Swerve.getInstance();

  // Return auto selected in Shuffleboard
  public static void runAutoCommand(AutoType auto) {

    ArrayList<ChoreoTrajectory> traj = Choreo.getTrajectoryGroup(auto.name);

    PIDController xController = new PIDController(5, 0, 0);
    PIDController yController = new PIDController(5, 0, 0);
    PIDController thetaController = new PIDController(2, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    ArrayList<Command> commandsToSchedule = new ArrayList<Command>();

    s_Swerve.resetOdometry(traj.get(0).getInitialPose());
    for(int i = 0; i < traj.size(); i++){
      Command swerveCommand = Choreo.choreoSwerveCommand(
      traj.get(i), 
      s_Swerve::getPose, 
      xController,
      yController,
      thetaController,
      (ChassisSpeeds speeds) -> s_Swerve.autoDrive(speeds, false), //this has to be robot-relative, need to check that auto-drive function works for this (may have to use drive function and set field-relative to false idk)
      () -> { Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red; }, //decides whether or not the math should be mirrored (depends on alliance)
      s_Swerve);
      commandsToSchedule.add(swerveCommand);
      commandsToSchedule.add(auto.mechCommands[i]);
      //CommandScheduler.getInstance().schedule(new SequentialCommandGroup(swerveCommand, auto.mechCommands[i]));
    }
    SequentialCommandGroup group = new SequentialCommandGroup(null);
    for (Command i : commandsToSchedule) {
      group.addCommands(i);
    }
    CommandScheduler.getInstance().schedule(group);

      // return swerveCommand;
  }

  public enum AutoType {
      TEST("test", new Command[]{new SetPivot(PivotState.GROUND), new SetPivot(PivotState.GROUND)}),
      ONEBALLAMP("one ball amp", new Command[]{}),
      BALLSPEAKER("ball speaker", new Command[]{});

      String name;
      Command[] mechCommands;

      private AutoType(String a, Command[] mechCommands){
        name = a;
        this.mechCommands = mechCommands;
      }
  }
}