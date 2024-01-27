// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TossCommand extends Command {
  private final ShooterSubsystem s_shooter;
  private boolean finish = false;
  private static boolean running = false;

  public TossCommand() {
    s_shooter = ShooterSubsystem.getInstance();
    addRequirements(s_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (running) {
      s_shooter.setVoltage(0);
    } else {
      s_shooter.setVoltage(10);
    }
  }

  @Override
  public void end(boolean interrupted) {
    running = !running;
  }

  @Override
  public boolean isFinished() {
    return finish;
  }
}
