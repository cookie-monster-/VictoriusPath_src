package org.usfirst.frc.team4587.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.usfirst.frc.team4587.robot.util.Gyro;
import org.usfirst.frc.team4587.robot.Constants;
import org.usfirst.frc.team4587.robot.GeneratedMotionProfile;
import org.usfirst.frc.team4587.robot.OI;
import org.usfirst.frc.team4587.robot.RobotMap;
//import com.team254.frc2017.Kinematics;
//import com.team254.frc2017.RobotState;
//import com.team254.frc2017.ShooterAimingParameters;
import org.usfirst.frc.team4587.robot.loops.Loop;
import org.usfirst.frc.team4587.robot.loops.Looper;
import org.usfirst.frc.team4587.robot.util.DriveSignal;
import org.usfirst.frc.team4587.robot.util.ReflectingCSVWriter;
//import com.team254.lib.util.Util;
//import com.team254.lib.util.control.Lookahead;
import org.usfirst.frc.team4587.robot.util.control.Path;
import org.usfirst.frc.team4587.robot.util.control.PathFollower;
//import org.usfirst.frc.team4587.robot.util.CANTalonFactory;
import org.usfirst.frc.team4587.robot.util.math.RigidTransform2d;
import org.usfirst.frc.team4587.robot.util.math.Rotation2d;
import org.usfirst.frc.team4587.robot.util.math.Twist2d;

import java.util.Arrays;
import java.util.Optional;

/**
 * This subsystem consists of the robot's drivetrain: 4 CIM motors, 4 talons, one solenoid and 2 pistons to shift gears,
 * and a navX board. The Drive subsystem has several control methods including open loop, velocity control, and position
 * control. The Drive subsystem also has several methods that handle automatic aiming, autonomous path driving, and
 * manual control.
 * 
 * @see Subsystem.java
 */
public class Drive extends Subsystem {

    private static Drive mInstance = null;
    private DifferentialDrive _drive;

    public static Drive getInstance() {
    	if(mInstance == null)
    	{
    		mInstance = new Drive();
    	}
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // used for autonomous driving
    }

    /**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(DriveControlState state) {
        if (state == DriveControlState.PATH_FOLLOWING) {
            return true;
        }
        return false;
    }

    /**
     * Check if the drive talons are configured for position control
     */
    protected static boolean usesTalonPositionControl(DriveControlState state) {
        return false;
    }

    // Control states
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;

    // Hardware
    private final WPI_TalonSRX mLeftMaster, mRightMaster;
    private final WPI_VictorSPX _leftSlave1, _leftSlave2, _rightSlave1, _rightSlave2;
    private final Gyro mNavXBoard;

    // Controllers
 //   private RobotState mRobotState = RobotState.getInstance();
    private PathFollower mPathFollower;

    // These gains get reset below!!
//    private Rotation2d mTargetHeading = new Rotation2d();
    private Path mCurrentPath = null;

    // Hardware states
    private boolean mIsBrakeMode;
    private boolean mStartingPath = false;
    private boolean mBufferOk = false;

    private MotionProfileStatus mLeftStatus = new MotionProfileStatus();
    private MotionProfileStatus mRightStatus = new MotionProfileStatus();

    // Logging
    private final ReflectingCSVWriter<PathFollower.DebugOutput> mCSVWriter;
    public void startPath() {
    	System.out.println("in startPath");
    	synchronized (Drive.this) {
    		mDriveControlState = DriveControlState.PATH_FOLLOWING;
    		mStartingPath = true ;
    	}
    }
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
            	
                setOpenLoop(DriveSignal.NEUTRAL);
                setBrakeMode(false);
                mNavXBoard.reset();
            }
        }

        @Override
        public void onLoop(double timestamp) {
        	iCall++;
        	if(iCall % 1000 == 0){
        		System.out.println("onLoop " + iCall + " " + mDriveControlState + " " + mLeftMaster.getControlMode());
        	}
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                case OPEN_LOOP:
                	_drive.arcadeDrive(OI.getInstance().getDrive(), OI.getInstance().getTurn());
                    //mLeftMaster.setInverted(false);
                    mRightMaster.setInverted(false);
                    _rightSlave1.setInverted(false);
                    _rightSlave2.setInverted(false);
                    _drive.setSafetyEnabled(true);
                    return;
                case PATH_FOLLOWING:
                    //mLeftMaster.setInverted(true);
                    mRightMaster.setInverted(true);
                    _rightSlave1.setInverted(true);
                    _rightSlave2.setInverted(true);
                    _drive.setSafetyEnabled(false);
                	doPathFollowing();
                   // if (mPathFollower != null) {
                   //     updatePathFollower(timestamp);
                   //     mCSVWriter.add(mPathFollower.getDebug());
                //    }
                    return;
                default:
                    System.out.println("Unexpected drive control state: " + mDriveControlState);
                    break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            mCSVWriter.flush();
        }
    };
    
    private TrajectoryDuration GetTrajectoryDuration(int durationMs)
	{	 
		/* create return value */
		TrajectoryDuration retval = TrajectoryDuration.Trajectory_Duration_0ms;
		/* convert duration to supported type */
		retval = retval.valueOf(durationMs);
		/* check that it is valid */
		if (retval.value != durationMs) {
			DriverStation.reportError("Trajectory Duration not supported - use configMotionProfileTrajectoryPeriod instead", false);		
		}
		/* pass to caller */
		return retval;
	}
    
	class PeriodicRunnable implements java.lang.Runnable {
	    public void run() { 
	    	mLeftMaster.processMotionProfileBuffer();
	    	mRightMaster.processMotionProfileBuffer();
	    }
	}
	Notifier _notifier = null;
	

    private void doPathFollowing () {
    	if(iCall % 1000 == 0){
    		System.out.println("doPathFollowing " + iCall); 
    	}
    	if (mStartingPath) {
    		mStartingPath = false;
    		mBufferOk = false;
    		mLeftMaster.clearMotionProfileHasUnderrun(0);
    		mRightMaster.clearMotionProfileHasUnderrun(0);
    		mLeftMaster.clearMotionProfileTrajectories();
    		mRightMaster.clearMotionProfileTrajectories();
    		mLeftMaster.configMotionProfileTrajectoryPeriod(0, 10);
    		mRightMaster.configMotionProfileTrajectoryPeriod(0,10);
    		mLeftMaster.setSelectedSensorPosition(0,0,10);
    		mRightMaster.setSelectedSensorPosition(0,0,10);
    		TrajectoryPoint point = new TrajectoryPoint();
    		
    		double vel = 180.0;
    		startFilling();
    		/*
    		for (int i = 0; i < 100; ++i) {
    			if(i > 90){
    				vel = 180.0 - (30 * 2/3 * (i-90));
    			}
    			point.position = i*.03*4096;
    			point.velocity = vel * 4096 / 600.0; //Convert RPM to Units/100ms
    			point.headingDeg = 0; 
    			point.profileSlotSelect0 = 0; 
    			point.profileSlotSelect1 = 0;
    			point.timeDur = GetTrajectoryDuration(10);
    			point.zeroPos = false;
    			if (i == 0)
    				point.zeroPos = true; 

    			point.isLastPoint = false;
    			
    			if ((i + 1) == 100){
    				point.isLastPoint = true;
    				point.velocity = 0;
    			}
    			mLeftMaster.pushMotionProfileTrajectory(point);
    			//point.position = -1 * point.position;
    			//point.velocity = -1 * point.velocity;
    			
    			mRightMaster.pushMotionProfileTrajectory(point);
    		}
    	*/
    	}
    	mLeftMaster.getMotionProfileStatus(mLeftStatus);
    	mRightMaster.getMotionProfileStatus(mRightStatus);
    	
    	if((mBufferOk == false) &&  (mLeftStatus.btmBufferCnt < 50 || mRightStatus.btmBufferCnt < 50)){
    		mLeftMaster.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
    		mRightMaster.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
        	if(iCall % 1000 == 0){
        		System.out.println("disabled");
        	}
    	}else{
    		mBufferOk = true;
    		int leftPos = mLeftMaster.getActiveTrajectoryPosition();
    		int rightPos = mRightMaster.getActiveTrajectoryPosition();
    		int leftEnc = mLeftMaster.getSelectedSensorPosition(0);
    		int rightEnc = mRightMaster.getSelectedSensorPosition(0);
    		
    		if(mLeftStatus.activePointValid && mLeftStatus.isLast){
        		mLeftMaster.set(ControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
            	if(iCall % 1000 == 0){
            		System.out.println("hold" + "L: " + leftPos + " " + leftEnc);
            		
            	}
    		}else{
        		mLeftMaster.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
            	if(iCall % 10 == 0){
            		System.out.println("enabled" + "L: " + leftPos + " " + leftEnc+" "+mLeftMaster.getMotorOutputPercent());
            	}
    		}
    		if(mRightStatus.activePointValid && mRightStatus.isLast){
        		mRightMaster.set(ControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
            	if(iCall % 1000 == 0){
            		System.out.println("hold" + "R: " + rightPos + " " + rightEnc);
            	}
    		}else{
        		mRightMaster.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
            	if(iCall % 10 == 0){
            		System.out.println("enable" + "R: " + rightPos + " " + rightEnc+" "+mRightMaster.getMotorOutputPercent());
            		
            	}
    		}
    	}
    }

    private Drive() {
        // Start all Talons in open loop mode.
        mLeftMaster = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_TALON);
        mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        mLeftMaster.setSensorPhase(true);
        mLeftMaster.changeMotionControlFramePeriod(5);
        
		mLeftMaster.configNeutralDeadband(0.01, 10);


		/* Our profile uses 10ms timing */
		mLeftMaster.configMotionProfileTrajectoryPeriod(10, 10); 
		/*
		 * status 10 provides the trajectory target for motion profile AND
		 * motion magic
		 */
		mLeftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
        
/*        mLeftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mLeftMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        mLeftMaster.reverseSensor(true);
        mLeftMaster.reverseOutput(false);
       WPI_TalonSRX.FeedbackDeviceStatus leftSensorPresent = mLeftMaster
                .isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (leftSensorPresent != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect left encoder: " + leftSensorPresent, false);
        }
        */

        _leftSlave1 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_VICTOR_1);
        _leftSlave1.follow(mLeftMaster);
/*        mLeftSlave.reverseOutput(false);
        mLeftMaster.setStatusFrameRateMs(StatusFrameRate.Feedback, 5);
*/
        _leftSlave2 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_VICTOR_2);
        _leftSlave2.follow(mLeftMaster);
        
        mRightMaster = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_TALON);
        mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        mRightMaster.changeMotionControlFramePeriod(5);
        mRightMaster.setSensorPhase(true);
        
		mRightMaster.configNeutralDeadband(0.01, 10);

		mRightMaster.config_kF(0, 0.275, 10);
		mRightMaster.config_kP(0, 0.25, 10);
		mRightMaster.config_kI(0, 0.0, 10);
		mRightMaster.config_kD(0, 0.0, 10);

		mLeftMaster.config_kF(0, 0.275, 10);
		mLeftMaster.config_kP(0, 0.25 , 10);
		mLeftMaster.config_kI(0, 0.0, 10);
		mLeftMaster.config_kD(0, 0.0, 10);
		/* Our profile uses 10ms timing */
		mRightMaster.configMotionProfileTrajectoryPeriod(10, 10); 
		/*
		 * status 10 provides the trajectory target for motion profile AND
		 * motion magic
		 */
		mRightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
      
/*        mRightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mRightMaster.reverseSensor(false);
        mRightMaster.reverseOutput(true);
        mRightMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        CANTalon.FeedbackDeviceStatus rightSensorPresent = mRightMaster
                .isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (rightSensorPresent != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect right encoder: " + rightSensorPresent, false);
        }
*/

        _rightSlave1 = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT_VICTOR_1);
        _rightSlave1.follow(mRightMaster);
/*        mRightSlave.reverseOutput(false);
        mRightMaster.setStatusFrameRateMs(StatusFrameRate.Feedback, 5);
*/
        _rightSlave2 = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT_VICTOR_2);
        _rightSlave2.follow(mRightMaster);
        

/*        mLeftMaster.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_10Ms);
        mLeftMaster.SetVelocityMeasurementWindow(32);
        mRightMaster.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_10Ms);
        mRightMaster.SetVelocityMeasurementWindow(32);
*/
        reloadGains();

        setOpenLoop(DriveSignal.NEUTRAL);

        // Path Following stuff
        mNavXBoard = new Gyro();

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mCSVWriter = new ReflectingCSVWriter<PathFollower.DebugOutput>("/home/lvuser/PATH-FOLLOWER-LOGS.csv",
                PathFollower.DebugOutput.class);
        
        _drive = new DifferentialDrive(mLeftMaster, mRightMaster);
    	_notifier = new Notifier(new PeriodicRunnable());
        _notifier.startPeriodic(0.005);
        
        
    }
    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
            mRightMaster.set(ControlMode.PercentOutput, -signal.getRight());
            mDriveControlState = DriveControlState.OPEN_LOOP;
            setBrakeMode(false);
        }
        else
        {
	        mRightMaster.set(-signal.getRight());
	        mLeftMaster.set(signal.getLeft());
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
//            mRightMaster.enableBrakeMode(on);
//            mRightSlave.enableBrakeMode(on);
//            mLeftMaster.enableBrakeMode(on);
//            mLeftSlave.enableBrakeMode(on);
        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() {
        final double left_speed = getLeftVelocityInchesPerSec();
        final double right_speed = getRightVelocityInchesPerSec();
//        SmartDashboard.putNumber("left voltage (V)", mLeftMaster.getOutputVoltage());
//        SmartDashboard.putNumber("right voltage (V)", mRightMaster.getOutputVoltage());
        SmartDashboard.putNumber("left speed (ips)", left_speed);
        SmartDashboard.putNumber("right speed (ips)", right_speed);
        SmartDashboard.putNumber("left percent output", mLeftMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("right percent output", mRightMaster.getMotorOutputPercent());
        if (usesTalonVelocityControl(mDriveControlState)) {
//          SmartDashboard.putNumber("left speed error (ips)",
//                    rpmToInchesPerSecond(mLeftMaster.getSetpoint()) - left_speed);
//            SmartDashboard.putNumber("right speed error (ips)",
//                    rpmToInchesPerSecond(mRightMaster.getSetpoint()) - right_speed);
        } else {
            SmartDashboard.putNumber("left speed error (ips)", 0.0);
            SmartDashboard.putNumber("right speed error (ips)", 0.0);
        }
        synchronized (this) {
            if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
                SmartDashboard.putNumber("drive CTE", mPathFollower.getCrossTrackError());
                SmartDashboard.putNumber("drive ATE", mPathFollower.getAlongTrackError());
            } else {
                SmartDashboard.putNumber("drive CTE", 0.0);
                SmartDashboard.putNumber("drive ATE", 0.0);
            }
        }
        SmartDashboard.putNumber("left position (rotations)", mLeftMaster.getSelectedSensorPosition(0));///4096);
        SmartDashboard.putNumber("right position (rotations)", mRightMaster.getSelectedSensorPosition(0));///4096);
//        SmartDashboard.putNumber("gyro vel", getGyroVelocityDegreesPerSec());
        SmartDashboard.putNumber("gyro pos", getGyroAngle().getDegrees());
    }

    public synchronized void resetEncoders() {
/*        mLeftMaster.setEncPosition(0);
        mLeftMaster.setPosition(0);
        mRightMaster.setPosition(0);
        mRightMaster.setEncPosition(0); 
        mLeftSlave.setPosition(0);
        mRightSlave.setPosition(0); */
    }

    @Override
    public void zeroSensors() {
        resetEncoders();
        mNavXBoard.reset();
    }

    /**
     * Configures talons for position control
     */
    private void configureTalonsForPositionControl() {
/*        if (!usesTalonPositionControl(mDriveControlState)) {
            // We entered a position control state.
            mLeftMaster.changeControlMode(CANTalon.TalonControlMode.MotionMagic);
            mLeftMaster.setNominalClosedLoopVoltage(12.0);
            mLeftMaster.setProfile(kLowGearPositionControlSlot);
            mLeftMaster.configNominalOutputVoltage(Constants.kDriveLowGearNominalOutput,
                    -Constants.kDriveLowGearNominalOutput);
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.MotionMagic);
            mRightMaster.setNominalClosedLoopVoltage(12.0);
            mRightMaster.setProfile(kLowGearPositionControlSlot);
            mRightMaster.configNominalOutputVoltage(Constants.kDriveLowGearNominalOutput,
                    -Constants.kDriveLowGearNominalOutput);
            setBrakeMode(true);
        } */
    }

    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (usesTalonVelocityControl(mDriveControlState)) {
            final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
                    ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;
            mLeftMaster.set(inchesPerSecondToRpm(left_inches_per_sec * scale));
            mRightMaster.set(inchesPerSecondToRpm(right_inches_per_sec * scale));
        } else {
            System.out.println("Hit a bad velocity control state");
            mLeftMaster.set(0);
            mRightMaster.set(0);
        }
    }

    /**
     * Adjust position setpoint (if already in position mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updatePositionSetpoint(double left_position_inches, double right_position_inches) {
        if (usesTalonPositionControl(mDriveControlState)) {
            mLeftMaster.set(inchesToRotations(left_position_inches));
            mRightMaster.set(inchesToRotations(right_position_inches));
        } else {
            System.out.println("Hit a bad position control state");
            mLeftMaster.set(0);
            mRightMaster.set(0);
        }
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    public double getLeftDistanceInches() {
        return rotationsToInches(mLeftMaster.getSelectedSensorPosition(0));
    }

    public double getRightDistanceInches() {
        return rotationsToInches(mRightMaster.getSelectedSensorPosition(0));
    }

    public double getLeftVelocityInchesPerSec() {
        return 0;//rpmToInchesPerSecond(mLeftMaster.getSpeed());
    }

    public double getRightVelocityInchesPerSec() {
        return 0;//rpmToInchesPerSecond(mRightMaster.getSpeed());
    }

    public synchronized Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(mNavXBoard.getYaw());
    }

    public synchronized Gyro getNavXBoard() {
        return mNavXBoard;
    }

/*    public synchronized void setGyroAngle(Rotation2d angle) {
        mNavXBoard.reset();
        mNavXBoard.setAngleAdjustment(angle);
    }*/

 /*   public synchronized double getGyroVelocityDegreesPerSec() {
        return mNavXBoard.getYawRateDegreesPerSec();
    } *
    
    /**
     * Essentially does the same thing as updateTurnToHeading but sends the robot into the DRIVE_TOWARDS_GOAL_APPROACH
     * state if it detects we are not at an optimal shooting range
     */

    /**
     * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
     * pose, distance driven, and velocity, the updates the wheel velocity setpoints.
     */
    private void updatePathFollower(double timestamp) {/*
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose,
                RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
        if (!mPathFollower.isFinished()) {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updateVelocitySetpoint(setpoint.left, setpoint.right);
        } else {
            updateVelocitySetpoint(0, 0);
        }*/
    }
    

    /**
     * Configures the drivebase for auto driving
     */


    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     * 
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {/*
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            configureTalonsForSpeedControl();
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(
                            new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                    Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
                            Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                            Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                            Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                            Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
                            Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
                            Constants.kPathStopSteeringDistance));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setVelocitySetpoint(0, 0);
        }*/
    }

    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    public synchronized void reloadGains() {/*
        mLeftMaster.setPID(Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi,
                Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf,
                Constants.kDriveLowGearPositionIZone, Constants.kDriveLowGearPositionRampRate,
                kLowGearPositionControlSlot);
        mLeftMaster.setMotionMagicCruiseVelocity(Constants.kDriveLowGearMaxVelocity);
        mLeftMaster.setMotionMagicAcceleration(Constants.kDriveLowGearMaxAccel);
        mRightMaster.setPID(Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi,
                Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf,
                Constants.kDriveLowGearPositionIZone, Constants.kDriveLowGearPositionRampRate,
                kLowGearPositionControlSlot);
        mRightMaster.setMotionMagicCruiseVelocity(Constants.kDriveLowGearMaxVelocity);
        mRightMaster.setMotionMagicAcceleration(Constants.kDriveLowGearMaxAccel);
        mLeftMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);
        mRightMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);

        mLeftMaster.setPID(Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone, Constants.kDriveHighGearVelocityRampRate,
                kHighGearVelocityControlSlot);
        mRightMaster.setPID(Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone, Constants.kDriveHighGearVelocityRampRate,
                kHighGearVelocityControlSlot);
        mLeftMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);
        mRightMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate); */
    }
    
    private void startFilling() {
		/* since this example only has one talon, just update that one */
		startFilling(GeneratedMotionProfile.Points, GeneratedMotionProfile.kNumPoints);
	}

	private void startFilling(double[][] profile, int totalCnt) {

		/* create an empty point */
		TrajectoryPoint point = new TrajectoryPoint();
		
		/* This is fast since it's just into our TOP buffer */
		for (int i = 0; i < totalCnt; ++i) {
			double positionRot = profile[i][0];
			double velocityRPM = profile[i][1];
			/* for each point, fill our structure and pass it to API */
			point.position = positionRot * Constants.kSensorUnitsPerRotation; //Convert Revolutions to Units
			point.velocity = velocityRPM * Constants.kSensorUnitsPerRotation / 600.0; //Convert RPM to Units/100ms
			point.headingDeg = 0; /* future feature - not used in this example*/
			point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
			point.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
			point.timeDur = GetTrajectoryDuration((int)profile[i][2]);
			point.zeroPos = false;
			if (i == 0)
				point.zeroPos = true; /* set this to true on the first point */

			point.isLastPoint = false;
			if ((i + 1) == totalCnt)
				point.isLastPoint = true; /* set this to true on the last point  */

			mRightMaster.pushMotionProfileTrajectory(point);
			mLeftMaster.pushMotionProfileTrajectory(point);
		}
	}

       @Override
    public void writeToLog() {
        mCSVWriter.write();
    }
}
