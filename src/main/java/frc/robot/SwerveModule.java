/* 
 swerve module objects, used by the swerve drivetrain library we use (team 364)
 https://github.com/Team364/BaseFalconSwerve
*/
package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.lib.math.FalconConversions;
import frc.lib.math.NEOConversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;


import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class SwerveModule {

    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkFlex mAngleMotor;
    private CANSparkFlex mDriveMotor;
    private CANcoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "2976 CANivore");
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkFlex(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            mDriveMotor.setControl(dutyCycleRequest.withOutput(percentOutput));
        }
        else {
            double velocity = FalconConversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
            mDriveMotor.setControl(velocityDutyCycle.withVelocity(velocity));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.setControl(positionDutyCycleRequest.withPosition(FalconConversions.degreesToFalcon(angle.getDegrees(), Constants.SwerveConstants.angleGearRatio)));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(FalconConversions.CANcoderToDegrees(angleEncoder.getPosition().getValueAsDouble(), Constants.SwerveConstants.angleEncoderGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    private void resetToAbsolute(){
        double absolutePosition = NEOConversions.degreesToNEO(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.SwerveConstants.angleGearRatio);
        // check later no idea if this is right
        mAngleMotor.getEncoder().setPosition(absolutePosition);
    }

    private void configAngleEncoder(){    
        angleEncoder.getConfigurator().apply(CTREConfigs.swerveCanCoderConfig);
    }   

    private void configAngleMotor(){
        mAngleMotor.setSmartCurrentLimit(Constants.SwerveConstants.anglePeakCurrentLimit);
        mAngleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        mAngleMotor.setIdleMode(IdleMode.kBrake);
        resetToAbsolute();
    }

    private void configDriveMotor(){   
        mDriveMotor.setSmartCurrentLimit(Constants.SwerveConstants.drivePeakCurrentLimit);
        mDriveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert);
        mDriveMotor.setIdleMode(IdleMode.kBrake);
        
        mDriveMotor.setOpenLoopRampRate(Constants.SwerveConstants.openLoopRamp);
        mDriveMotor.setClosedLoopRampRate(Constants.SwerveConstants.closedLoopRamp);

        mDriveMotor.getEncoder().setPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            FalconConversions.falconToMPS(mDriveMotor.getEncoder().getVelocity(), Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio), 

            getAngle()
        ); 
    }

    // //added by Yanda to try to debug the drifting by comparing actual bus voltage.
    // public double getDriveBusVoltage(){
    //     return mDriveMotor.getBusVoltage();
    // }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            FalconConversions.falconToMeters(mDriveMotor.getEncoder().getPosition(), Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio),
            getAngle()
        );
    }
}