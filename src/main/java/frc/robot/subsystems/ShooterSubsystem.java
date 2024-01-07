// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Constants.Ports;

public class ShooterSubsystem extends SubsystemBase {
  
  private static ShooterSubsystem instance = null;
  public static ShooterSubsystem getInstance() {
        if(instance == null){
            instance = new ShooterSubsystem();
        }
        return instance;
    }

  private final LazyTalonFX mMasterMotor, mFollowerMotor;

  private ShooterSubsystem() {
      mMasterMotor = TalonFXFactory.createDefaultFalcon("Master Shooter Motor", Ports.MASTER_SHOOTER_MOTOR);
      configureMotor(mMasterMotor, true);
      mFollowerMotor = TalonFXFactory.createSlaveFalcon("Follower Shooter Motor", Ports.FOLLOW_SHOOTER_MOTOR, mMasterMotor);
      mFollowerMotor.setLeader(mMasterMotor);
      configureMotor(mFollowerMotor, false);
      //setMultipleStatuFramePeriod();
  }

  PIDController shooterController;

  private void configureMotor(LazyTalonFX talon, boolean b){
      talon.setInverted(b);
      talon.setControl(new VoltageOut(12.0));
      talon.setNeutralMode(NeutralModeValue.Coast);
      //talon.config_kF(0.05, Constants.kTimeOutMs);
      talon.config_kP(0.12, Constants.kTimeOutMs);
      talon.config_kI(0, Constants.kTimeOutMs);
      talon.config_kD(0, Constants.kTimeOutMs);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
