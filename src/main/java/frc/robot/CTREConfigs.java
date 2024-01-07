/* 
 swerve CTRE configs, used by the swerve drivetrain library we use (team 364)
 https://github.com/Team364/BaseFalconSwerve
*/

package frc.robot;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

public final class CTREConfigs {
    public static TalonFXConfiguration swerveAngleFXConfig;
    public static TalonFXConfiguration swerveDriveFXConfig;
    public static CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();
        Slot0Configs slot0Configs = new Slot0Configs();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

        /* Swerve Angle Motor Configurations */
        currentLimitsConfigs.SupplyCurrentLimit = Constants.SwerveConstants.angleContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentThreshold = Constants.SwerveConstants.anglePeakCurrentLimit;
        slot0Configs.kP = Constants.SwerveConstants.angleKP;
        slot0Configs.kI = Constants.SwerveConstants.angleKI;
        slot0Configs.kD = Constants.SwerveConstants.angleKD;
        swerveAngleFXConfig.Slot0 = slot0Configs;
        swerveAngleFXConfig.CurrentLimits = currentLimitsConfigs;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.angleNeutralMode;
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.SwerveConstants.angleMotorInvert;

        /* Swerve Drive Motor Configuration */
        currentLimitsConfigs.SupplyCurrentLimit = Constants.SwerveConstants.driveContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentThreshold = Constants.SwerveConstants.drivePeakCurrentLimit;
        slot0Configs.kP = Constants.SwerveConstants.driveKP;
        slot0Configs.kI = Constants.SwerveConstants.driveKI;
        slot0Configs.kD = Constants.SwerveConstants.driveKD;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.driveNeutralMode;
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.SwerveConstants.driveMotorInvert;
        
        swerveDriveFXConfig.Slot0 = slot0Configs;
        swerveDriveFXConfig.CurrentLimits = currentLimitsConfigs;
        
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
        openLoopRampsConfigs.TorqueOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
        openLoopRampsConfigs.VoltageOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;

        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
        closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
        closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;

        swerveDriveFXConfig.OpenLoopRamps = openLoopRampsConfigs;
        swerveDriveFXConfig.ClosedLoopRamps = closedLoopRampsConfigs;
        
        /* Swerve CANCoder Configuration */
        // CANcoder is always initialized to absolute position on boot in Phoenix 6 - https://www.chiefdelphi.com/t/what-kind-of-encoders-are-built-into-the-kraken-motors/447253/7

        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.SensorDirection = Constants.SwerveConstants.canCoderInvert;
        
        swerveCanCoderConfig.MagnetSensor = magnetSensorConfigs;
    }
}