
package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;


public class SwerveModule {

    // Control requests
    // Reuse these, don't make more
    final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    final VoltageOut voltageOutRequest = new VoltageOut(0);
    // ControlMode.Position without voltage compensation
    final PositionDutyCycle positionDutyCycleRequest = new PositionDutyCycle(0);
    // ControlMode.Position with voltage compensation
    final PositionVoltage positionVoltageRequest = new PositionVoltage(0);
    // ControlMode.Velocity without voltage compensation
    final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);
    // ControlMode.Velocity with voltage compensation
    final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "2976 CANivore");
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "2976 CANivore");
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "2976 CANivore");
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
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);
            mDriveMotor.setControl(velocityDutyCycle.withVelocity(velocity));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.setControl(positionDutyCycleRequest.withPosition(Conversions.degreesToFalcon(angle.getDegrees(), Constants.SwerveConstants.angleGearRatio)));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.CANcoderToDegrees(angleEncoder.getPosition().getValueAsDouble(), Constants.SwerveConstants.angleEncoderGearRatio));    
    }
    
    /**
     * Gets the Rotation2d of recorded by the CANcoder.
     * @return Rotation2d recorded by the CANcoder.
     */
    public Rotation2d getCanCoderAngle(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    private void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoderAngle().getDegrees() - angleOffset.getDegrees(), Constants.SwerveConstants.angleGearRatio);
        // check later no idea if this is right
        mAngleMotor.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){    
        angleEncoder.getConfigurator().apply(CTREConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.getConfigurator().apply(CTREConfigs.swerveAngleFXConfig);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.getConfigurator().apply(CTREConfigs.swerveDriveFXConfig);
        mDriveMotor.setPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio), 
            getAngle()
        ); 
    }

    // //added by Yanda to try to debug the drifting by comparing actual bus voltage.
    // public double getDriveBusVoltage(){
    //     return mDriveMotor.getBusVoltage();
    // }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio), 
            getAngle()
        );
    }
}