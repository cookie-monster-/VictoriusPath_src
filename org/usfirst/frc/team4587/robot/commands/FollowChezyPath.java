package org.usfirst.frc.team4587.robot.commands;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team4587.robot.Robot;
import org.usfirst.frc.team4587.robot.paths.TestPath;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4587.robot.util.Gyro;
import jaci.pathfinder.Trajectory;

/**
 *
 */
public class FollowChezyPath extends Command {

	BufferedReader m_bufferedReader;
	boolean quit;
	int m_startEncoderLeft;
	int m_startEncoderRight;
	double m_startAngle;
	double m_startTime;
	double Ka = 0.0001;//0.01;//0.1
	double Kv = 0.000275;
	double Kp = 0.00025;
	double Kg = 0.0001;//0.015;
	Trajectory leftPath;
	Trajectory rightPath;
	FileWriter m_logWriter;
	String m_namePath;
	String filename;
	boolean m_backwards;
	boolean m_reverseLeftRight;
	double m_finalPositionRight;
	double m_finalPositionLeft;
	double m_gyroMultiplier;
    public FollowChezyPath(String namePath) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	m_namePath = namePath;
    	filename = "/home/lvuser/" + m_namePath + ".txt";
    }

    // Called just before this Command runs the first time
    public void initialize() {
    	leftPath = Robot.getTestPath().left;
    	rightPath = Robot.getTestPath().right;
    	quit = false;
    	try{
    		//m_bufferedReader = new BufferedReader(new FileReader(filename));
    		StringBuilder sb = new StringBuilder();
    		String line;
    		while((line = m_bufferedReader.readLine()) != null)
    		{
    			sb.append(line).append("\n");
    		}
    		m_bufferedReader.close();
    	}catch(Exception e){
    		System.out.println(e);
    	}

		try {
			m_logWriter = new FileWriter("/home/lvuser/" + m_namePath +"Log.csv", false);
			m_logWriter.write("aLeft,vLeft,xLeft,aRight,vRight,xRight,desiredAngle,currentAngle,realLeftEncoder,realRightEncoder,leftMotorLevel,rightMotorLevel,System.nanoTime()" + "\n");
		} catch ( IOException e ) {
			System.out.println(e);
			m_logWriter = null;
		}
    	m_startEncoderLeft = Robot.getDrive().getLeftEnc();
    	m_startEncoderRight = Robot.getDrive().getRightEnc();
    	m_startTime = System.nanoTime();
    	
    	m_finalPositionLeft = Robot.getTestPath().left.get(Robot.getTestPath().left.length()-1).position * 12 / 0.0046;
    	m_finalPositionRight = Robot.getTestPath().right.get(Robot.getTestPath().right.length()-1).position * 12 / 0.0046;
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
    	System.out.println("FollowChezyPath.execute()");
    	double time = System.nanoTime();
    	double dt = (time - m_startTime) / 1000000;
    	int step0 = (int)(dt / 10);
    	int step1 = step0 + 1;
    	double offset = dt - 10 * step0;
    	
    	double aLeft;
		double vLeft;
		double xLeft;
		double aRight;
		double vRight;
		double xRight;
    	
    	if(step1 >= leftPath.length())
    	{
    		quit = true;
    		Robot.getDrive().setMotorLevels(0.0, 0.0);
    	}
    	/*else if(m_backwards && m_finalPositionRight >= Robot.getDriveBaseSimple().getEncoderRight() && m_finalPositionLeft >= Robot.getDriveBaseSimple().getEncoderLeft()){
    		quit = true;
    	}
    	else if(m_backwards == false && m_finalPositionRight <= Robot.getDriveBaseSimple().getEncoderRight() && m_finalPositionLeft <= Robot.getDriveBaseSimple().getEncoderLeft()){
    		quit = true;
    	}*/
    	else
        	{
	    		Trajectory.Segment left0;
	        	Trajectory.Segment right0;
	    		Trajectory.Segment left1;
	        	Trajectory.Segment right1;
	        	
            	left0 = leftPath.get(step0);
            	left1 = leftPath.get(step1);
            	right0 = rightPath.get(step0);
            	right1 = rightPath.get(step1);
    			
            	aLeft = (left0.acceleration + ((offset / 10) * (left1.acceleration - left0.acceleration))) * 12 / 0.0046 / 1000 * 10 / 1000 * 10;
	        	vLeft = (left0.velocity + ((offset / 10) * (left1.velocity - left0.velocity))) * 12 / 0.0046 / 1000 * 10;
	        	xLeft = (left0.position + ((offset / 10) * (left1.position - left0.position))) * 12 / 0.0046;
	        	aRight = (right0.acceleration + ((offset / 10) * (right1.acceleration - right0.acceleration))) * 12 / 0.049 / 1000 * 10 / 1000 * 10;
	        	vRight = (right0.velocity + ((offset / 10) * (right1.velocity - right0.velocity))) * 12 / 0.0046 / 1000 * 10;
	        	xRight = (right0.position + ((offset / 10) * (right1.position - right0.position))) * 12 / 0.0046;

        		double desiredAngle = right0.heading * 180 / Math.PI * m_gyroMultiplier; //* -1;
        		double currentAngle = Gyro.getYaw();
        		int realLeftEncoder = Robot.getDrive().getLeftEnc();
        		int realRightEncoder = Robot.getDrive().getRightEnc();
        		desiredAngle += m_startAngle;
        		while(desiredAngle > 180)
        		{
        			desiredAngle -= 360;
        		}
        		while(desiredAngle < -180)
        		{
        			desiredAngle += 360;
        		}
        		xLeft += m_startEncoderLeft;
        		xRight += m_startEncoderRight;
        		double leftMotorLevel = Ka * aLeft + Kv * vLeft - Kp * (realLeftEncoder - xLeft) - Kg * (currentAngle - desiredAngle);
        		double rightMotorLevel = Ka * aRight + Kv * vRight - Kp * (realRightEncoder - xRight) + Kg * (currentAngle - desiredAngle);
        		
        		Robot.getDrive().setMotorLevels(leftMotorLevel, -rightMotorLevel);
        		SmartDashboard.putNumber("left motor set to: ", leftMotorLevel);
        		SmartDashboard.putNumber("right motor set to: ", -rightMotorLevel);
            	
        		if(m_logWriter != null)
        		{
        			try{
        				m_logWriter.write(aLeft + "," + vLeft + "," + xLeft + "," + aRight + "," + vRight + "," + xRight + "," + desiredAngle + "," + currentAngle + "," + realLeftEncoder + "," + realRightEncoder + "," + leftMotorLevel + "," + rightMotorLevel + "," + time + "\n");
        			}catch(Exception e){
        				
        			}
        		}
        	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return quit;
    }

    // Called once after isFinished returns true
    protected void end() {
    	try
    	{
    		m_logWriter.close();
    	}
    	catch(Exception e)
    	{
    		
    	}
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
