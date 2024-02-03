// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArnavIndexer extends SubsystemBase {

private static ArnavIndexer instance;


    public static ArnavIndexer getInstance() {
        if (instance == null) {
            instance = new ArnavIndexer();
        }
        return instance;
    }
  
private String state;

private CANSparkMax indexerM;

  public ArnavIndexer() {
      indexerM = new CANSparkMax(Constants.HardwarePorts.indexerM, MotorType.kBrushless);
  }

  public enum IndexerStates {
  ON(1),
  OFF(0);
  private double speed;

  IndexerStates(double speed) {
    this.speed = speed;
  }
}

  public void setSpeed(IndexerStates state) {
    indexerM.set(state.speed);
    this.state = state.name();
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putString("indexer state", state);
  }

  @Override
  public void simulationPeriodic() {
  }
}
