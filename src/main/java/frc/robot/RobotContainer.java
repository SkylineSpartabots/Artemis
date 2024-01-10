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
  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);

  /* Driver Joysticks (drive control) */

  /* Driver Buttons */
  private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
  
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

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.b().whileTrue(new TossCommand());
  }
  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(s_exampleSubsystem);
  }
}
