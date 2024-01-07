// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.FlywheelSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TossCommand extends Command {
  private final FlywheelSubsystem s_flywheel;

  public TossCommand() {
    s_flywheel = FlywheelSubsystem.getInstance();
    addRequirements(s_flywheel);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    s_flywheel.setVoltage(10);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
