// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem extends SubsystemBase {
  
  private static ShooterSubsystem instance;
  public static ShooterSubsystem getInstance() {
    if (instance == null)
        instance = new ShooterSubsystem();
    return instance;
  }

  private TalonFX shooterLeaderM;
  private TalonFX shooterFollowerM;

  private double voltage;
  
  public ShooterSubsystem() {
    shooterLeaderM = new TalonFX(Constants.HardwarePorts.shooterLeaderM);
    shooterFollowerM = new TalonFX(Constants.HardwarePorts.shooterFollowerM);
    shooterFollowerM.setInverted(true);
    shooterFollowerM.setControl(new StrictFollower(Constants.HardwarePorts.shooterLeaderM));
  }


  public void setVoltage(double voltage) {
    this.voltage = voltage;
    shooterLeaderM.setVoltage(voltage);
  }

  public double getVoltageSetpoint() {
    return voltage;
  }
 
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Voltage Setpoint", getVoltageSetpoint());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
