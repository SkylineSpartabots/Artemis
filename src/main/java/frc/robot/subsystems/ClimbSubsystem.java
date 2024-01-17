// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  private static ClimbSubsystem instance;

  public static ClimbSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimbSubsystem();
    }
    return instance;
  }
  
  //current expected design: single telescoping arm with hook
  private TalonFX mClimb;
  private TalonSRX mPivot; //using phoenix5 api for SRX; phoenix6 discontinued it

  private ClimbSubsystem() {
    mClimb = new TalonFX(Constants.HardwarePorts.climbMotor);
    mClimb.setNeutralMode(NeutralModeValue.Brake);

    mPivot = new TalonSRX(Constants.HardwarePorts.climbPivotMotor);
    mPivot.setNeutralMode(NeutralMode.Brake);
    mPivot.neutralOutput();
  }

  public void setVoltage(double voltage) {
    mClimb.setVoltage(voltage);
  }

  public void setPivotPercentPower(double power) {
    mPivot.set(ControlMode.PercentOutput, power); //SRX has no voltage control mode
  }

  @Override
  public void periodic() {
  }
}
