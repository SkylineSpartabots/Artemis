// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PivotSubsystem.PivotState;

public class SetPivot extends Command {
  private final PivotSubsystem pivotSubsystem;
  private PivotSubsystem.PivotState state;
  private double pivotVoltage;
  private PIDController pivotController = new PIDController(0, 0, 0); //TODO set to real values

  public SetPivot(PivotState state) {
      pivotSubsystem = PivotSubsystem.getInstance();
      addRequirements(pivotSubsystem);
      this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotSubsystem.setState(state);
    pivotController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotVoltage = pivotController.calculate(pivotSubsystem.getCANCoderPosition(), pivotSubsystem.getCANCoderSetpoint());
    pivotSubsystem.setVoltage(pivotVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
