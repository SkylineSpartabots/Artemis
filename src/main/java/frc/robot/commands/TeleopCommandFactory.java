package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.Swerve;

public class TeleopCommandFactory extends Command{
    private static Command selectedTeleopCommand;

    private static IntakeShooter s_IntakeShooter = IntakeShooter.getInstance();
    private static Swerve s_Swerve = Swerve.getInstance();

    public TeleopCommandFactory(){
        
    }
}