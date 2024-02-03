
package frc.robot.commands;

import frc.robot.subsystems.ArnavIndexer;
import frc.robot.subsystems.ArnavIntake;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SetIndexer extends Command {
  private final ArnavIndexer s_ArnavIndexer;
  ArnavIndexer.IndexerStates state;
  
  public SetIndexer(ArnavIndexer.IndexerStates state) {
    s_ArnavIndexer = ArnavIndexer.getInstance();
    this.state = state;
    addRequirements(s_ArnavIndexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    s_ArnavIndexer.setSpeed(state);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}