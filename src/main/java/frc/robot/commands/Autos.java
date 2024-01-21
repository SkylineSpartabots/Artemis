// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {  
      
  private static Command lastCommand;
  private static Command selectedAuto;


  // Return auto selected in Shuffleboard
  public static Command getAutoCommand(AutoType auto) {
      switch (auto) {
      case Test:
          return selectedAuto = test();
      case Test2:
          return selectedAuto = test2();
      case Test3:
          return selectedAuto = test3();
      default:
          break;
      }
      return null;
  }

  public enum AutoType {
      Test,
      Test2,
      Test3
  }

  public static Command getSelectedAuto() {
    return selectedAuto;
  }

  public static void cancelLastCommand() {
    lastCommand.cancel();
  }


  private static Command test() {
    return null;
  }
  
  private static Command test2() {
    return null;
  }

  private static Command test3() {
    return null;
  }

  private Autos() {
      throw new UnsupportedOperationException("This is a utility class!");
  }
}
