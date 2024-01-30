package frc.robot.subsystems;

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

    private enum PivotState {
        GROUND(0);

        private double pos;

        private PivotState(double position) {
            pos = position;
        }
    }

    private CANSparkFlex mPivotMotor;

    public Pivot() {
        mPivotMotor = new CANSparkFlex(Constants.HardwarePorts.pivotMotor, MotorType.kBrushless);
        configMotor();
    }
    
    public void setPosition(PivotState state) {
        mPivotMotor.getEncoder().setPosition(state.pos);
    }

    private void configMotor() {
        mPivotMotor.setSmartCurrentLimit(Constants.pivotPeakCurrentLimit);
        mPivotMotor.setIdleMode(IdleMode.kBrake);

        mPivotMotor.getPIDController().setP(Constants.hardwarePIDs.pivotkP);
        mPivotMotor.getPIDController().setI(Constants.hardwarePIDs.pivotkI);
        mPivotMotor.getPIDController().setD(Constants.hardwarePIDs.pivotkD);
    }
    
}
