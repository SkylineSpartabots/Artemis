// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Swerve s_Swerve = Swerve.getInstance();
  private final IntakeShooter s_IntakeShooter = IntakeShooter.getInstance();
  private final Pivot s_Pivot = Pivot.getInstance();
  private final Vision s_Vision = Vision.getInstance();

  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);
  
  /* Driver Joysticks (drive control) */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton driverBack = new JoystickButton(driver, XboxController.Button.kBack.value);
  private final JoystickButton driverStart = new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton driverRightBumper = new JoystickButton(driver,
          XboxController.Button.kRightBumper.value);
  private final JoystickButton driverLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  private final Trigger driverDpadUp = new Trigger(() -> driver.getPOV() == 0);
  private final Trigger driverDpadRight = new Trigger(() -> driver.getPOV() == 90);
  private final Trigger driverDpadDown = new Trigger(() -> driver.getPOV() == 180);
  private final Trigger driverDpadLeft = new Trigger(() -> driver.getPOV() == 270);
  private final Trigger driverLeftTrigger = new Trigger(() -> {
      return driver.getLeftTriggerAxis() > Constants.triggerDeadzone;
  });
  private final Trigger driverRightTrigger = new Trigger(() -> {
      return driver.getRightTriggerAxis() > Constants.triggerDeadzone;
  });

  /* Operator Buttons */
  private final JoystickButton operatorBack = new JoystickButton(operator, XboxController.Button.kBack.value);
  private final JoystickButton operatorStart = new JoystickButton(operator, XboxController.Button.kStart.value);
  private final JoystickButton operatorA = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton operatorB = new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton operatorX = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton operatorY = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton operatorRightBumper = new JoystickButton(operator,
          XboxController.Button.kRightBumper.value);
  private final JoystickButton operatorLeftBumper = new JoystickButton(operator,
          XboxController.Button.kLeftBumper.value);

  private final Trigger operatorDpadUp = new Trigger(() -> operator.getPOV() == 0);
  private final Trigger operatorDpadRight = new Trigger(() -> operator.getPOV() == 90);
  private final Trigger operatorDpadDown = new Trigger(() -> operator.getPOV() == 180);
  private final Trigger operatorDpadLeft = new Trigger(() -> operator.getPOV() == 270);
  private final Trigger operatorLeftTrigger = new Trigger(() -> {
      return operator.getLeftTriggerAxis() > Constants.triggerDeadzone;
  });
  private final Trigger operatorRightTrigger = new Trigger(() -> {
      return operator.getRightTriggerAxis() > Constants.triggerDeadzone;
  });

  /* Operator Joysticks */
  private final int operatorLeftStick = XboxController.Axis.kLeftY.value;
  private final int operatorRightStick = XboxController.Axis.kRightY.value;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.resetOdometry(new Pose2d());
    s_Swerve.zeroGyro();

    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, 
    () -> -driver.getRawAxis(translationAxis),
    () -> -driver.getRawAxis(strafeAxis),
    () -> -driver.getRawAxis(rotationAxis)));

    // Configure the trigger bindings
    configureBindings();
  }

  /* Map commands to buttons */
  private void configureBindings() {
    // driver controls
    driverBack.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())));

    driverA.onTrue(new InstantCommand(() -> s_IntakeShooter.setPercentage(1.0)));
    driverB.onTrue(new InstantCommand(() -> s_IntakeShooter.setPercentage(0)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
