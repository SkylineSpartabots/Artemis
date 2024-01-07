// TODO: Migrate TalonFXFactory.java to Phoenix 6

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and BACK_RIGHTared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class TalonFXFactory {

    public final static double kTimeoutMs = Constants.kTimeOutMs;

    public static class Configuration {
        public NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
        public double NEUTRAL_DEADBAND = 0.04;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;

        public InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

        //public TalonFXFeedbackDevice DEVICE = TalonFXFeedbackDevice.IntegratedSensor;

        public double INPUT_CURRENT_LIMIT = 20;
        public double OUTPUT_CURRENT_LIMIT = 20;

        //public MotorCommutation COMMUTATION = MotorCommutation.Trapezoidal;

        //public SensorInitializationStrategy INITIALIZATION_STRATEGY = SensorInitializationStrategy.BootToZero;
        public boolean SENSOR_PHASE = false;
        public double SENSOR_FEEDBACK_COEFFECIENT = 1; // TODO: Calculate new values for ratio.
        //OLD: for gear reduction x (input) -> y (output), coeffecient = 256 * x / Math.PI * y;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

        //public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_WINDOW = 64;

    }

    private static Configuration kDefaultConfiguration = new Configuration();
    private static Configuration kSlaveConfiguration = new Configuration();

    static {
        kSlaveConfiguration.CONTROL_FRAME_PERIOD_MS = 25;
        kSlaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 35;
        kSlaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 5000;
        kSlaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
        // kSlaveConfiguration.DEVICE = TalonFXFeedbackDevice.None;
    }

    public static LazyTalonFX createDefaultFalcon(String name, int deviceID) {
        return createFalcon(name, deviceID, kDefaultConfiguration);
    }

    public static LazyTalonFX createSlaveFalcon(String name, int deviceID, LazyTalonFX leader) {
        final LazyTalonFX falcon = createFalcon(name, deviceID, kSlaveConfiguration);
        falcon.setLeader(leader);
        return falcon;
    }

    public static LazyTalonFX createFalcon(String name, int deviceID, Configuration config) {
        LazyTalonFX falcon = new LazyTalonFX(name, deviceID);
        falcon.setControl(new DutyCycleOut(0.0));
        
        TalonFXConfiguration configs = new TalonFXConfiguration();

        //falcon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        //falcon.clearMotionProfileHasUnderrun();
        //falcon.clearMotionProfileTrajectories();
        configs.MotorOutput.withDutyCycleNeutralDeadband(config.NEUTRAL_DEADBAND);

        //PheonixUtil.checkError(falcon.configNeutralDeadband(), name +
        //     " failed to configure neutral deadband on init", false);
        
        configs.MotorOutput.withPeakReverseDutyCycle(-1.0);
        configs.MotorOutput.withPeakForwardDutyCycle(1.0);
        //falcon.configNominalOutputForward(0.0);
        //falcon.configNominalOutputReverse(0.0);

        //falcon.configVoltageCompSaturation(0.0);
        //falcon.configVoltageMeasurementFilter(32);
        //falcon.enableVoltageCompensation(false);

        //falcon.selectProfileSlot(0, 0);

        configs.CurrentLimits.StatorCurrentLimit = config.OUTPUT_CURRENT_LIMIT;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = config.OUTPUT_CURRENT_LIMIT;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // PheonixUtil.checkError(falcon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, kTimeoutMs), name +
        //      " failed to set velocity meas. period on init", true);

        // PheonixUtil.checkError(falcon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_WINDOW, kTimeoutMs), name +
        //      " failed to set velocity measurement window on init", true);    
      
        falcon.clearStickyFaults();

        /*PheonixUtil.checkError(falcon.configSelectedFeedbackSensor(config.DEVICE, 0, kTimeoutMs),
            name + " failed to set feedback sensor on init", true);*/

        //
        configs.Feedback.SensorToMechanismRatio = config.SENSOR_FEEDBACK_COEFFECIENT;
        
        // Possibly related to configs.configs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod.
        // PheonixUtil.checkError(falcon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, kTimeoutMs), 
        //     name + " failed to set open loop ramp rate on init", true);
        
        // PheonixUtil.checkError(falcon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, kTimeoutMs), 
        //     name + " failed to set closed loop ramp rate on init", true);

        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        
        configs.MotorOutput.Inverted = config.INVERTED;

        //falcon.setSensorPhase(config.SENSOR_PHASE);
    
        return falcon;
    }

}