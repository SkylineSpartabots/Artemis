// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

public class FlywheelSubsystem extends SubsystemBase {
  
  private static FlywheelSubsystem instance;
  public static FlywheelSubsystem getInstance() {
    if (instance == null)
        instance = new FlywheelSubsystem();
    return instance;
  }

  private TalonFX flywheelLeaderM;
  private TalonFX flywheelFollowerM;

  private double voltage;
  
  public FlywheelSubsystem() {
    flywheelLeaderM = new TalonFX(Constants.HardwarePorts.flywheelLeaderM);
    flywheelFollowerM = new TalonFX(Constants.HardwarePorts.flywheelFollowerM);
    flywheelFollowerM.setInverted(true);
    flywheelFollowerM.setControl(new StrictFollower(Constants.HardwarePorts.flywheelLeaderM));
  }


  public void setVoltage(double voltage) {
    this.voltage = voltage;
    flywheelLeaderM.setVoltage(voltage);
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
