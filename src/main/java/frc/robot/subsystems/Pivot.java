package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
    public static Pivot instance;
    public static Pivot getInstance() {
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }

    public enum PivotState {
        GROUND(0);

        private double pos;

        private PivotState(double position) {
            pos = position;
        }
    }

    private CANSparkFlex mPivotMotor;
    private CANcoder pivotCANcoder;
    private PivotState curState = PivotState.GROUND;

    public Pivot() {
        mPivotMotor = new CANSparkFlex(Constants.HardwarePorts.pivotMotor, MotorType.kBrushless);
        configMotor();

        pivotCANcoder = new CANcoder(Constants.HardwarePorts.pivotCANcoderID);
        configCANcoder();
    }

    public void setState(PivotState state) {
        curState = state;
    }

    public double getCANcoderPosition() {
        return pivotCANcoder.getPosition().getValueAsDouble();
    }

    public double getSetPoint() {
        return curState.pos;
    }

    public void setVoltage(double voltage) {
        mPivotMotor.setVoltage(voltage);
    }

    private void configMotor() {
        mPivotMotor.setSmartCurrentLimit(Constants.pivotPeakCurrentLimit);
        mPivotMotor.setIdleMode(IdleMode.kBrake);

        mPivotMotor.getPIDController().setP(Constants.hardwarePIDs.pivotkP);
        mPivotMotor.getPIDController().setI(Constants.hardwarePIDs.pivotkI);
        mPivotMotor.getPIDController().setD(Constants.hardwarePIDs.pivotkD);
    }
    
    private void configCANcoder(){  
        /* Swerve CANCoder Configuration */
        // CANcoder is always initialized to absolute position on boot in Phoenix 6 - https://www.chiefdelphi.com/t/what-kind-of-encoders-are-built-into-the-kraken-motors/447253/7

        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        
        CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();
        swerveCanCoderConfig.MagnetSensor = magnetSensorConfigs;  
        pivotCANcoder.getConfigurator().apply(swerveCanCoderConfig);
    }   
}
