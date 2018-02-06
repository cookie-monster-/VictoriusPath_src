/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4587.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	private static OI mInstance = null;
	private Joystick stick1;
	
	public OI()
	{
		stick1 = new Joystick(1);
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
