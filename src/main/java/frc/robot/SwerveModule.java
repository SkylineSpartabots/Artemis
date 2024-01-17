
package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class SwerveModule {

    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    // private CANSparkFlex mAngleMotor;
    // private CANSparkFlex mDriveMotor;
    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private CANcoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        // mAngleMotor = new CANSparkFlex(moduleConstants.angleMotorID, MotorType.kBrushless);
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);

        configAngleMotor();

        /* Drive Motor Config */
        // mDriveMotor = new CANSparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless); //change back later
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);

        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Sets the speed of the drive motor
     * @param desiredState Desired set state for PID Controller
     * @param isOpenLoop   Whether to use open loop or close loop control
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.MPSToMotor(desiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
            mDriveMotor.getPIDController().setReference(velocity, ControlType.kVelocity);
        }
    }
    /**
     * Sets the rotation of the angle motor
     * @param desiredState Desired set state  for PID Controller
     */
    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.getPIDController().setReference(Conversions.degreesToMotor(angle.getDegrees(), Constants.SwerveConstants.angleGearRatio), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.CANcoderToDegrees(angleEncoder.getPosition().getValueAsDouble(), Constants.SwerveConstants.angleEncoderGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    private void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToMotor(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.SwerveConstants.angleGearRatio);
        mAngleMotor.getEncoder().setPosition(absolutePosition);
    }

    private void configAngleEncoder(){  
        /* Swerve CANCoder Configuration */
        // CANcoder is always initialized to absolute position on boot in Phoenix 6 - https://www.chiefdelphi.com/t/what-kind-of-encoders-are-built-into-the-kraken-motors/447253/7

        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.SensorDirection = Constants.SwerveConstants.canCoderInvert;
        
        CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();
        swerveCanCoderConfig.MagnetSensor = magnetSensorConfigs;  
        angleEncoder.getConfigurator().apply(swerveCanCoderConfig);
    }   

    private void configAngleMotor(){
        mAngleMotor.setSmartCurrentLimit(Constants.SwerveConstants.anglePeakCurrentLimit);
        mAngleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);

        mAngleMotor.getPIDController().setP(Constants.SwerveConstants.angleKP);
        mAngleMotor.getPIDController().setI(Constants.SwerveConstants.angleKI);
        mAngleMotor.getPIDController().setD(Constants.SwerveConstants.angleKD);

        resetToAbsolute();
    }

    private void configDriveMotor(){   
        mDriveMotor.setSmartCurrentLimit(Constants.SwerveConstants.drivePeakCurrentLimit);
        mDriveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
        
        mDriveMotor.setOpenLoopRampRate(Constants.SwerveConstants.openLoopRamp);
        mDriveMotor.setClosedLoopRampRate(Constants.SwerveConstants.closedLoopRamp);

        mDriveMotor.getPIDController().setP(Constants.SwerveConstants.driveKP);
        mDriveMotor.getPIDController().setI(Constants.SwerveConstants.driveKI);
        mDriveMotor.getPIDController().setD(Constants.SwerveConstants.driveKD);

        mDriveMotor.getEncoder().setPosition(0);
    }
    /**
     * Gets the current velocity and rotation of the module.
     * @return A SwerveModuleState containing the current velocity and rotation of the module.
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.motorToMPS(mDriveMotor.getEncoder().getVelocity(), Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio), 
            getAngle()
        ); 
    }

    /**
     * Gets the current distance measured by the wheel and rotation of the module.
     * @return A SwerveModulePosition containing the current measured distance and rotation of the module.
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.motorToMeters(mDriveMotor.getEncoder().getPosition(), Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio),
            getAngle()
        );
    }
}