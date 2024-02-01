// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.IntakeShooter;

//for future reference: blue speaker ID = 7, red speaker ID = 4
public class AutoAimShoot extends Command {
  Vision s_Vision;
  Pivot s_Pivot;
  IntakeShooter s_IntakeShooter;
  public AutoAimShoot() {
    s_IntakeShooter = IntakeShooter.getInstance();
    addRequirements(s_IntakeShooter);
    s_Vision = Vision.getInstance();
    addRequirements(s_Vision);
    s_Pivot = Pivot.getInstance();
    addRequirements(s_Pivot);
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
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !s_IntakeShooter.hasNote();
  }
}
