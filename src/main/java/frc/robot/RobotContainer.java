// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TossCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /* Driver Joysticks (drive control) */

  /* Driver Buttons */

  
  /* Operator Buttons */

  /* Operator Sticks */

  /* Subsystems */
  private final ShooterSubsystem s_shooterSubsystem;
  private final ExampleSubsystem s_exampleSubsystem = new ExampleSubsystem();
      
  public RobotContainer() {
    
    // Initialize subsystems
    s_shooterSubsystem = ShooterSubsystem.getInstance();
    
    
    // Configure the button bindings
    configureBindings();
  }

  /* Map commands to buttons */
  private void configureBindings() {
    
    // driver controls
    driver.a().whileTrue(new TossCommand());

    // operator controls

  }
  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(s_exampleSubsystem);
  }
}
