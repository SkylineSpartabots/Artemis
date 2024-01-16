package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    private CANSparkMax motor;

    private IntakeSubsystem() {
        motor = new CANSparkMax(Constants.HardwarePorts.intakeMotor, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kCoast);
    }

    private enum IntakeState {
        // TODO figure out if we need to be at 100% or we can be at something lower
        ON(1),
        OFF(0);
        private double speed;
        IntakeState(double speed) {
            this.speed = speed;
        }
    }
    private void setSpeed(IntakeState state) {
        motor.set(state.speed);
    }
}