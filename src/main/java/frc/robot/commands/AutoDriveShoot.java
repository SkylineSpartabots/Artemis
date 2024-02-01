// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve;

//for future reference: blue speaker ID = 7, red speaker ID = 4
public class AutoDriveShoot extends Command {
  Vision s_Vision;
  Swerve s_Swerve;
  public AutoDriveShoot() {
    s_Vision = Vision.getInstance();
    addRequirements(s_Vision);
    s_Swerve = Swerve.getInstance();
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
