/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4587.robot;

import org.usfirst.frc.team4587.robot.commands.startTeleopDrive;
import org.usfirst.frc.team4587.robot.commands.startTestPath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
//import utility.JoyButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	private static OI mInstance = null;
	private Joystick stick1;
	Button	  buttonA1, buttonB1, buttonX1, buttonY1, leftBumper1, rightBumper1;
	//JoyButton leftTrigger1, rightTrigger1;
	Joystick  stick2;
	Button	  buttonA2, buttonB2, buttonX2, buttonY2, leftBumper2, rightBumper2;
	//JoyButton leftTrigger2, rightTrigger2;
	
	public OI()
	{
		//stick1 = new Joystick(1);
		stick1			= new Joystick(1);
    	buttonA1		= new JoystickButton(stick1, 1);
    	buttonB1		= new JoystickButton(stick1, 2);
    	buttonX1		= new JoystickButton(stick1, 3);
    	buttonY1		= new JoystickButton(stick1, 4);
    	leftBumper1 	= new JoystickButton(stick1, 5);
    	//leftTrigger1	= new JoyButton(stick1, JoyButton.JoyDir.DOWN, 2);
    	rightBumper1	= new JoystickButton(stick1, 6);
    	//rightTrigger1	= new JoyButton(stick1, JoyButton.JoyDir.DOWN, 3);
    	
    	stick2			= new Joystick(2);
    	buttonA2		= new JoystickButton(stick2, 1);
    	buttonB2		= new JoystickButton(stick2, 2);
    	buttonX2		= new JoystickButton(stick2, 3);
    	buttonY2		= new JoystickButton(stick2, 4);
    	leftBumper2 	= new JoystickButton(stick2, 5);
    	//leftTrigger2	= new JoyButton(stick2, JoyButton.JoyDir.DOWN, 2);
    	rightBumper2	= new JoystickButton(stick2, 6);
    	//rightTrigger2	= new JoyButton(stick2, JoyButton.JoyDir.DOWN, 3);
    	
    	System.out.println("OI start");
    	buttonA1.whenPressed(new startTeleopDrive());
    	buttonB1.whenPressed(new startTestPath());
	}
	
	public double getTurn()
	{
		return stick1.getRawAxis(4);
	}
	
	public static OI getInstance()
	{
		if(mInstance == null)
		{
			mInstance = new OI();
		}
		return mInstance;
	}
	
	public double getDrive()
	{
		return -1 * stick1.getRawAxis(1);
	}
}
