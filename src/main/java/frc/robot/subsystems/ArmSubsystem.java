package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {
    private static ArmSubsystem instance;

    public static ArmSubsystem getInstance() {
        if (instance == null) {
            instance = new ArmSubsystem();
        }
        return instance;
    }

    private CANSparkMax leaderMotor;
    private CANSparkMax followerMotor;
    private CANcoder canCoder;
    private ArmState state = ArmState.FLOOR;

    private ArmSubsystem() {
        leaderMotor = new CANSparkMax(Constants.HardwarePorts.armLeaderMotor, MotorType.kBrushless);
        leaderMotor.setIdleMode(IdleMode.kBrake);
        followerMotor = new CANSparkMax(Constants.HardwarePorts.armFollowerMotor, MotorType.kBrushless);
        followerMotor.follow(leaderMotor);
        canCoder = new CANcoder(Constants.HardwarePorts.armCanCoder);
    }

    // TODO set to some real value, these is totally random
    public enum ArmState {
        FLOOR(0.0),
        AMP(80.0),
        SPEAKER(120.0);
        final double position;

        ArmState(double position) {
            this.position = position;
        }
    }

    public void setState(ArmState state) {
        this.state = state;
    }

    public double getCANCoderPosition() {
        return canCoder.getPosition().getValueAsDouble();
    }

    public double getCANCoderSetpoint() {
        return state.position;
    }

    public void setVoltage(double voltage) {
        leaderMotor.setVoltage(voltage);
    }
}