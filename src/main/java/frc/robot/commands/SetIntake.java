
package frc.robot.commands;

import frc.robot.subsystems.ArnavIntake;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SetIntake extends Command {
  private final ArnavIntake s_ArnavIntake;
  ArnavIntake.IntakeStates state;
  
  public SetIntake(ArnavIntake.IntakeStates state) {
    s_ArnavIntake = ArnavIntake.getInstance();
    this.state = state;
    addRequirements(s_ArnavIntake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
        s_ArnavIntake.setSpeed(state);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}