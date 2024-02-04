
package frc.robot.commands;

import frc.robot.subsystems.ArnavShooter;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SetShooter extends Command {
  private final ArnavShooter s_ArnavShooter;
  double finalspeed;
  
  public SetShooter(ArnavShooter.ShooterStates state) { //change from off or max speed
    s_ArnavShooter = ArnavShooter.getInstance();
    finalspeed = state.getValue();
    addRequirements(s_ArnavShooter);
  }

  public SetShooter(double difference) { //increase or decrease speed. Makes sure not to increase above max or decrease below 0
    s_ArnavShooter = ArnavShooter.getInstance();

    double addedspeed = s_ArnavShooter.getspeed() + difference;

    if(addedspeed <= 1 && addedspeed >= 0) {
      finalspeed = addedspeed;
    }

    addRequirements(s_ArnavShooter);
  }

  @Override
  public void initialize() {
      
  }

  @Override
  public void execute() {
        s_ArnavShooter.setspeed(finalspeed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}