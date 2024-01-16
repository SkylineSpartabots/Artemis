package frc.robot.subsystems;

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

    private ArmSubsystem() {
        motor = new TalonFX(Constants.HardwarePorts.armMotor);
    }

    public enum ArmState {
        FLOOR(0.0),
        AMP(80.0), // TODO set to some real value, this is totally random
        SPEAKER(120.0); // TODO set to some real value, this is totally random
        final double position;

        ArmState(double position) {
            this.position = position;
        }
    }

    public void setState(ArmState state) {
        motor.setPosition(state.position);
    }
}