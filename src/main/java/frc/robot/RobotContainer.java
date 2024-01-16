// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TossCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /* Driver Joysticks (drive control) */

  /* Driver Buttons */

  
  /* Operator Buttons */

  /* Operator Sticks */

  /* Subsystems */
  private final ShooterSubsystem shooterSubsystem;
  private final ArmSubsystem armSubsystem;

  public RobotContainer() {
    
    // Initialize subsystems
    shooterSubsystem = ShooterSubsystem.getInstance();
    armSubsystem = ArmSubsystem.getInstance();
    
    
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
    return null; // TODO fix
  }
}
