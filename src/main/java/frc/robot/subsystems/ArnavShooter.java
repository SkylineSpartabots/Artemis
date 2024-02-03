// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArnavShooter extends SubsystemBase {

private static ArnavShooter instance;


    public static ArnavShooter getInstance() {
        if (instance == null) {
            instance = new ArnavShooter();
        }
        return instance;
    }

private CANSparkMax shooterLeaderM;
  private CANSparkMax shooterFollowerM;

  private double currentSpeed;

  public ArnavShooter() {
      shooterLeaderM = new CANSparkMax(Constants.HardwarePorts.shooterLeaderM, MotorType.kBrushless);
      shooterFollowerM = new CANSparkMax(Constants.HardwarePorts.shooterFollowerM, MotorType.kBrushless);
      shooterFollowerM.setInverted(true);
      shooterFollowerM.follow(shooterLeaderM);
  }

  public enum ShooterStates {
    MAX(1),
    OFF(0);
    private double voltage;
  
    public double getValue() {
      return voltage;
    }

    ShooterStates(double voltage) {
      this.voltage = voltage;
    }
    
  }

  public void setVoltage(ShooterStates state) { //change state
    shooterLeaderM.set(state.voltage);
    currentSpeed = state.getValue();
  }

  public void setVoltage(double newVoltage) { //change specific voltage
    shooterFollowerM.set(newVoltage);
    currentSpeed = newVoltage;
  }

  public double getVoltage() { //gets specific voltage (i hope)
    return currentSpeed;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current speed", currentSpeed);
  }

  @Override
  public void simulationPeriodic() {
  }
}
