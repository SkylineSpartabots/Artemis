package frc.robot.subsystems;

import javax.sound.midi.MidiEvent;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeShooter extends SubsystemBase {

    private static IntakeShooter mIntakeShooter;

    private CANSparkFlex mIntakeMotor;
    private CANSparkFlex mLeaderShooter;
    private CANSparkFlex mFollowerShooter;

    public static IntakeShooter getInstance(){
        if(mIntakeShooter == null){
            mIntakeShooter = new IntakeShooter();
        }
        return mIntakeShooter;
    }

    public IntakeShooter(){
        mIntakeMotor = new CANSparkFlex(Constants.HardwarePorts.intakeMotor, MotorType.kBrushless);
        configIntakeMotor();

        mLeaderShooter = new CANSparkFlex(Constants.HardwarePorts.shooterLeaderMotor, MotorType.kBrushless);
        configLeaderMotor();

        mFollowerShooter = new CANSparkFlex(Constants.HardwarePorts.shooterFollowerMotor, MotorType.kBrushless);
        configFollowerMotor();
    }

    public void setPercentage(double val){
        mLeaderShooter.set(val);
        mIntakeMotor.set(val);
    }

    public void configIntakeMotor(){
        mIntakeMotor.setSmartCurrentLimit(Constants.intakePeakCurrentLimit);
        mIntakeMotor.setInverted(false);
        mIntakeMotor.setIdleMode(Constants.shooterNeutralMode);
        mIntakeMotor.setOpenLoopRampRate(Constants.SwerveConstants.openLoopRamp);

    }

    public void configLeaderMotor(){
        mLeaderShooter.setSmartCurrentLimit(Constants.intakePeakCurrentLimit);
        mLeaderShooter.setInverted(true);
        mLeaderShooter.setIdleMode(Constants.shooterNeutralMode);
        mLeaderShooter.setOpenLoopRampRate(Constants.SwerveConstants.openLoopRamp);


        // mLeaderShooter.setOpenLoopRampRate(Constants.SwerveConstants.openLoopRamp);
        // mLeaderShooter.setClosedLoopRampRate(Constants.SwerveConstants.closedLoopRamp);

        mLeaderShooter.getPIDController().setP(Constants.hardwarePIDs.shooterkP);
        mLeaderShooter.getPIDController().setI(Constants.hardwarePIDs.shooterkI);
        mLeaderShooter.getPIDController().setD(Constants.hardwarePIDs.shooterkP);
    }

    public void configFollowerMotor(){
        mFollowerShooter.follow(mLeaderShooter, false);
        mFollowerShooter.setIdleMode(Constants.shooterNeutralMode);
        
        // mLeaderShooter.getPIDController().setP(Constants.SwerveConstants.driveKP);
        // mLeaderShooter.getPIDController().setI(Constants.SwerveConstants.driveKI);
        // mLeaderShooter.getPIDController().setD(Constants.SwerveConstants.driveKD);
    }
    
}
