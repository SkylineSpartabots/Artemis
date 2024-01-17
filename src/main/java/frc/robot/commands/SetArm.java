// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;

public class SetArm extends Command {
  private final ArmSubsystem s_arm;
  private ArmSubsystem.ArmState state;
  private double armVoltage;
  private PIDController armController = new PIDController(0, 0, 0); //TODO set to real values

  public SetArm(ArmState state) {
      s_arm = ArmSubsystem.getInstance();
      addRequirements(s_arm);
      this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_arm.setState(state);
    armController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armVoltage = armController.calculate(s_arm.getCANCoderPosition(), s_arm.getCANCoderSetpoint());
    s_arm.setVoltage(armVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_arm.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
