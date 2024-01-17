package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private static ArmSubsystem instance;

    public static ArmSubsystem getInstance() {
        if (instance == null) {
            instance = new ArmSubsystem();
        }
        return instance;
    }

    private TalonFX motor;
    private CANcoder canCoder;
    private ArmState state = ArmState.FLOOR;

    private ArmSubsystem() {
        motor = new TalonFX(Constants.HardwarePorts.armMotor);
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
        motor.setVoltage(voltage);
    }
}