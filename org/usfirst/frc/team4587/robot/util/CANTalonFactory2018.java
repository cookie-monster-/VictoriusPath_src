package org.usfirst.frc.team4587.robot.util;


import edu.wpi.first.wpilibj.MotorSafety;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class CANTalonFactory2018 {

    public static class Configuration {
    	public boolean INVERT_MOTOR = false;
    	public boolean INVERT_SENSOR = false;
    	
        public boolean LIMIT_SWITCH_NORMALLY_OPEN = true;
        public double MAX_OUTPUT_VOLTAGE = 12;
        public double NOMINAL_VOLTAGE = 0;
        public double PEAK_VOLTAGE = 12;
        public double PEAK_OUTPUT = 1;
        public double MAX_OUTPUT = 1;
        public double NOMINAL_OUTPUT = 0;//??????
        public boolean ENABLE_BRAKE = false;
        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int CURRENT_LIMIT = 0;
        public double EXPIRATION_TIMEOUT_SECONDS = MotorSafety.DEFAULT_SAFETY_EXPIRATION;
        public int FORWARD_SOFT_LIMIT = 0;
        public boolean INVERTED = false;
        public double NOMINAL_CLOSED_LOOP_VOLTAGE = 12;
        public double REVERSE_SOFT_LIMIT = 0;
        public boolean SAFETY_ENABLED = false;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 100;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 100;
        public int VOLTAGE_COMPENSATION_TIMEOUT_MS = 10;
        public int STANDARD_TIMEOUT_MS = 10;

        //public VelocityMeasurementPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasurementPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public double VOLTAGE_COMPENSATION_RAMP_RATE = 0;
        public double VOLTAGE_RAMP_RATE = 0;
        
/*?*/        public double OPEN_LOOP_RAMP_RATE = 0;
/*?*/        public double CLOSED_LOOP_RAMP_RATE = 0;
    }

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kSlaveConfiguration = new Configuration();

    static {
        kSlaveConfiguration.CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    }

    // Create a CANTalon with the default (out of the box) configuration.
    public static WPI_TalonSRX createDefaultTalon(int id) {
        return createTalon(id, kDefaultConfiguration);
    }

    public static WPI_VictorSPX createPermanentSlaveTalon(int id, WPI_TalonSRX master) {
        final WPI_VictorSPX victor = null;// createTalon(id, kSlaveConfiguration);
        victor.follow(master);
        return victor;
    }

    public static WPI_TalonSRX createTalon(int id, Configuration config) {
        WPI_TalonSRX talon = new WPI_TalonSRX(id);
        talon.enableVoltageCompensation(true);
        talon.configVoltageCompSaturation(config.MAX_OUTPUT_VOLTAGE, config.VOLTAGE_COMPENSATION_TIMEOUT_MS);
        talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        talon.setIntegralAccumulator(0,0,config.STANDARD_TIMEOUT_MS);
        talon.clearMotionProfileHasUnderrun(config.STANDARD_TIMEOUT_MS);
        talon.clearMotionProfileTrajectories();
        talon.clearStickyFaults(10);
        /*
 //?        * shouldn't need this because 254 was just setting to 0, and now it forces us to actually set a sensor source
        talon.ConfigFwdLimitSwitchNormallyOpen(config.LIMIT_SWITCH_NORMALLY_OPEN);
        talon.configForwardLimitSwitchSource();
        */
        talon.configPeakOutputForward(config.PEAK_OUTPUT,config.STANDARD_TIMEOUT_MS);
        talon.configPeakOutputReverse(-config.PEAK_OUTPUT,config.STANDARD_TIMEOUT_MS);
        
 //?       //next two NOMINAL_OUTPUT could be 0 or 1, not sure
        talon.configNominalOutputForward(config.NOMINAL_OUTPUT, config.STANDARD_TIMEOUT_MS);
        talon.configNominalOutputReverse(-config.NOMINAL_OUTPUT, config.STANDARD_TIMEOUT_MS);
        /*
 //?        * shouldn't need this because 254 was just setting to 0, and now it forces us to actually set a sensor source
        talon.ConfigRevLimitSwitchNormallyOpen(config.LIMIT_SWITCH_NORMALLY_OPEN);
        talon.configReverseLimitSwitchSource();
        */
        talon.setNeutralMode(NeutralMode.Brake);
        talon.enableCurrentLimit(config.ENABLE_CURRENT_LIMIT);
        talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT,config.STANDARD_TIMEOUT_MS);
        talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);
        talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT,config.STANDARD_TIMEOUT_MS);
        /*
 //?       talon.enableZeroSensorPositionOnForwardLimit(false);
 //?       talon.enableZeroSensorPositionOnIndex(false, false);
 //?       talon.enableZeroSensorPositionOnReverseLimit(false);
        */
        talon.setInverted(config.INVERT_MOTOR);
        talon.setSensorPhase(config.INVERT_SENSOR);
        talon.getSensorCollection().setAnalogPosition(0,config.STANDARD_TIMEOUT_MS);
        talon.configContinuousCurrentLimit(config.CURRENT_LIMIT,config.STANDARD_TIMEOUT_MS);
        talon.setExpiration(config.EXPIRATION_TIMEOUT_SECONDS);
        talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, config.STANDARD_TIMEOUT_MS);
        talon.setSelectedSensorPosition(0,0,config.STANDARD_TIMEOUT_MS);
        talon.selectProfileSlot(0,0);
        talon.getSensorCollection().setPulseWidthPosition(0,config.STANDARD_TIMEOUT_MS);
        talon.setSafetyEnabled(config.SAFETY_ENABLED);
   //?     //talon.configVelocityMeasurementPeriod();
        talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW,config.STANDARD_TIMEOUT_MS);
        talon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, config.STANDARD_TIMEOUT_MS);
        talon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, config.STANDARD_TIMEOUT_MS);

        talon.setStatusFramePeriod(StatusFrame.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, config.STANDARD_TIMEOUT_MS);
        talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.FEEDBACK_STATUS_FRAME_RATE_MS,config.STANDARD_TIMEOUT_MS);
   //?     talon.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, config.QUAD_ENCODER_STATUS_FRAME_RATE_MS);
        talon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS,config.STANDARD_TIMEOUT_MS);
                
   //?     talon.setStatusFrameRateMs(CANTalon.StatusFrameRate.PulseWidth, config.PULSE_WIDTH_STATUS_FRAME_RATE_MS);

        return talon;
    }

    /**
     * Run this on a fresh talon to produce good values for the defaults.
     */
    /*
    public static String getFullTalonInfo(WPI_TalonSRX talon) {
        StringBuilder sb = new StringBuilder().append("isRevLimitSwitchClosed = ")
                .append(talon.isRevLimitSwitchClosed()).append("\n").append("getBusVoltage = ")
                .append(talon.getBusVoltage()).append("\n").append("isForwardSoftLimitEnabled = ")
                .append(talon.isForwardSoftLimitEnabled()).append("\n").append("getFaultRevSoftLim = ")
                .append(talon.getFaultRevSoftLim()).append("\n").append("getStickyFaultOverTemp = ")
                .append(talon.getStickyFaultOverTemp()).append("\n").append("isZeroSensorPosOnFwdLimitEnabled = ")
                .append(talon.isZeroSensorPosOnFwdLimitEnabled()).append("\n")
                .append("getMotionProfileTopLevelBufferCount = ").append(talon.getMotionProfileTopLevelBufferCount())
                .append("\n").append("getNumberOfQuadIdxRises = ").append(talon.getNumberOfQuadIdxRises()).append("\n")
                .append("getInverted = ").append(talon.getInverted()).append("\n")
                .append("getPulseWidthRiseToRiseUs = ").append(talon.getPulseWidthRiseToRiseUs()).append("\n")
                .append("getError = ").append(talon.getError()).append("\n").append("isSensorPresent = ")
                .append(talon.isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative)).append("\n")
                .append("isControlEnabled = ").append(talon.isControlEnabled()).append("\n").append("getTable = ")
                .append(talon.getTable()).append("\n").append("isEnabled = ").append(talon.isEnabled()).append("\n")
                .append("isZeroSensorPosOnRevLimitEnabled = ").append(talon.isZeroSensorPosOnRevLimitEnabled())
                .append("\n").append("isSafetyEnabled = ").append(talon.isSafetyEnabled()).append("\n")
                .append("getOutputVoltage = ").append(talon.getOutputVoltage()).append("\n").append("getTemperature = ")
                .append(talon.getTemperature()).append("\n").append("getSmartDashboardType = ")
                .append(talon.getSmartDashboardType()).append("\n").append("getPulseWidthPosition = ")
                .append(talon.getPulseWidthPosition()).append("\n").append("getOutputCurrent = ")
                .append(talon.getOutputCurrent()).append("\n").append("get = ").append(talon.get()).append("\n")
                .append("isZeroSensorPosOnIndexEnabled = ").append(talon.isZeroSensorPosOnIndexEnabled()).append("\n")
                .append("getMotionMagicCruiseVelocity = ").append(talon.getMotionMagicCruiseVelocity()).append("\n")
                .append("getStickyFaultRevSoftLim = ").append(talon.getStickyFaultRevSoftLim()).append("\n")
                .append("getFaultRevLim = ").append(talon.getFaultRevLim()).append("\n").append("getEncPosition = ")
                .append(talon.getEncPosition()).append("\n").append("getIZone = ").append(talon.getIZone()).append("\n")
                .append("getAnalogInPosition = ").append(talon.getAnalogInPosition()).append("\n")
                .append("getFaultUnderVoltage = ").append(talon.getFaultUnderVoltage()).append("\n")
                .append("getCloseLoopRampRate = ").append(talon.getCloseLoopRampRate()).append("\n")
                .append("toString = ").append(talon.toString()).append("\n")
                // .append("getMotionMagicActTrajPosition =
                // ").append(talon.getMotionMagicActTrajPosition()).append("\n")
                .append("getF = ").append(talon.getF()).append("\n").append("getClass = ").append(talon.getClass())
                .append("\n").append("getAnalogInVelocity = ").append(talon.getAnalogInVelocity()).append("\n")
                .append("getI = ").append(talon.getI()).append("\n").append("isReverseSoftLimitEnabled = ")
                .append(talon.isReverseSoftLimitEnabled()).append("\n").append("getPIDSourceType = ")
                .append(talon.getPIDSourceType()).append("\n").append("getEncVelocity = ")
                .append(talon.getEncVelocity()).append("\n").append("GetVelocityMeasurementPeriod = ")
                .append(talon.GetVelocityMeasurementPeriod()).append("\n").append("getP = ").append(talon.getP())
                .append("\n").append("GetVelocityMeasurementWindow = ").append(talon.GetVelocityMeasurementWindow())
                .append("\n").append("getDeviceID = ").append(talon.getDeviceID()).append("\n")
                .append("getStickyFaultRevLim = ").append(talon.getStickyFaultRevLim()).append("\n")
                // .append("getMotionMagicActTrajVelocity =
                // ").append(talon.getMotionMagicActTrajVelocity()).append("\n")
                .append("getReverseSoftLimit = ").append(talon.getReverseSoftLimit()).append("\n").append("getD = ")
                .append(talon.getD()).append("\n").append("getFaultOverTemp = ").append(talon.getFaultOverTemp())
                .append("\n").append("getForwardSoftLimit = ").append(talon.getForwardSoftLimit()).append("\n")
                .append("GetFirmwareVersion = ").append(talon.GetFirmwareVersion()).append("\n")
                .append("getLastError = ").append(talon.getLastError()).append("\n").append("isAlive = ")
                .append(talon.isAlive()).append("\n").append("getPinStateQuadIdx = ").append(talon.getPinStateQuadIdx())
                .append("\n").append("getAnalogInRaw = ").append(talon.getAnalogInRaw()).append("\n")
                .append("getFaultForLim = ").append(talon.getFaultForLim()).append("\n").append("getSpeed = ")
                .append(talon.getSpeed()).append("\n").append("getStickyFaultForLim = ")
                .append(talon.getStickyFaultForLim()).append("\n").append("getFaultForSoftLim = ")
                .append(talon.getFaultForSoftLim()).append("\n").append("getStickyFaultForSoftLim = ")
                .append(talon.getStickyFaultForSoftLim()).append("\n").append("getClosedLoopError = ")
                .append(talon.getClosedLoopError()).append("\n").append("getSetpoint = ").append(talon.getSetpoint())
                .append("\n").append("isMotionProfileTopLevelBufferFull = ")
                .append(talon.isMotionProfileTopLevelBufferFull()).append("\n").append("getDescription = ")
                .append(talon.getDescription()).append("\n").append("hashCode = ").append(talon.hashCode()).append("\n")
                .append("isFwdLimitSwitchClosed = ").append(talon.isFwdLimitSwitchClosed()).append("\n")
                .append("getPinStateQuadA = ").append(talon.getPinStateQuadA()).append("\n")
                .append("getPinStateQuadB = ").append(talon.getPinStateQuadB()).append("\n").append("GetIaccum = ")
                .append(talon.GetIaccum()).append("\n").append("getFaultHardwareFailure = ")
                .append(talon.getFaultHardwareFailure()).append("\n").append("pidGet = ").append(talon.pidGet())
                .append("\n").append("getBrakeEnableDuringNeutral = ").append(talon.getBrakeEnableDuringNeutral())
                .append("\n").append("getStickyFaultUnderVoltage = ").append(talon.getStickyFaultUnderVoltage())
                .append("\n").append("getPulseWidthVelocity = ").append(talon.getPulseWidthVelocity()).append("\n")
                .append("GetNominalClosedLoopVoltage = ").append(talon.GetNominalClosedLoopVoltage()).append("\n")
                .append("getPosition = ").append(talon.getPosition()).append("\n").append("getExpiration = ")
                .append(talon.getExpiration()).append("\n").append("getPulseWidthRiseToFallUs = ")
                .append(talon.getPulseWidthRiseToFallUs()).append("\n")
                // .append("createTableListener = ").append(talon.createTableListener()).append("\n")
                .append("getControlMode = ").append(talon.getControlMode()).append("\n")
                .append("getMotionMagicAcceleration = ").append(talon.getMotionMagicAcceleration()).append("\n")
                .append("getControlMode = ").append(talon.getControlMode());

        return sb.toString();
    }
    */
}
