/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4587.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

import java.io.FileWriter;
import java.util.Arrays;

import org.usfirst.frc.team4587.robot.loops.Looper;
import org.usfirst.frc.team4587.robot.paths.TestPath;
import org.usfirst.frc.team4587.robot.subsystems.Drive;
import org.usfirst.frc.team4587.robot.util.CrashTracker;
import org.usfirst.frc.team4587.robot.util.DriveSignal;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

	 private SubsystemManager mSubsystemManager = null;
	 private Looper mEnabledLooper = null;
	 private static Drive mDrive = null;
	 public static Drive getDrive(){
		 return mDrive;
	 }
	 private static TestPath mTestPath;
	 public static TestPath getTestPath(){
		 return mTestPath;
	 }
	 private static FileWriter writer;
	 public static void writeToFile(String x){
		 try{
		 writer.write(x);
		 }catch(Exception e){}
	 }
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public Robot() {
	CrashTracker.logRobotConstruction();
	}
	@Override
	public void robotInit() {
		mEnabledLooper = new Looper();
		mSubsystemManager = new SubsystemManager(Arrays.asList(Drive.getInstance()));
		mDrive = Drive.getInstance();
		OI.getInstance();
		try {
			CrashTracker.logRobotInit();
		    mSubsystemManager.registerEnabledLoops(mEnabledLooper);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
		    throw t;
		}
		// 3 Waypoints
		mTestPath = new TestPath();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
	      try {
	            CrashTracker.logDisabledInit();

	            mEnabledLooper.stop();

	            // Call stop on all our Subsystems.
	            mSubsystemManager.stop();

	            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
	            
	        } catch (Throwable t) {
	            CrashTracker.logThrowableCrash(t);
	            throw t;
	        }
	      try{
	      writer.close();
	      }catch(Exception e){}
	}

	@Override
	public void disabledPeriodic() {
        allPeriodic();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		mEnabledLooper.start();
		mDrive.startPath();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		allPeriodic();
	}

	@Override
	public void teleopInit() {
		try {
            CrashTracker.logTeleopInit();

            // Start loopers
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            //mDrive.setBrakeMode(false);
            // Shift to high
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
		try{
	    writer = new FileWriter("/home/lvuser/PathLog.csv");
	    }catch(Exception e){}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
        try {
            double timestamp = Timer.getFPGATimestamp();
            
            allPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	 public void allPeriodic() {
	        mSubsystemManager.outputToSmartDashboard();
	        mSubsystemManager.writeToLog();
	        mEnabledLooper.outputToSmartDashboard();
			Scheduler.getInstance().run();
}
	}
	