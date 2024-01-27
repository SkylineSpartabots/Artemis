// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  
  private static ShooterSubsystem instance;
  public static ShooterSubsystem getInstance() {
    if (instance == null)
        instance = new ShooterSubsystem();
    return instance;
  }

  private CANSparkFlex m_shooterLeader;
  private CANSparkFlex m_shooterFollower;

  private double voltage;
  
  public ShooterSubsystem() {
    m_shooterLeader = new CANSparkFlex(Constants.HardwarePorts.shooterLeaderM, MotorType.kBrushless);
    m_shooterFollower = new CANSparkFlex(Constants.HardwarePorts.shooterFollowerM, MotorType.kBrushless);
    m_shooterFollower.setInverted(true);
    m_shooterFollower.follow(m_shooterLeader);
  }


  public void setVoltage(double voltage) {
    this.voltage = voltage;
    m_shooterLeader.setVoltage(voltage);
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
