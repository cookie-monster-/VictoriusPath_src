package org.usfirst.frc.team4587.robot.paths;

import java.io.File;
import java.io.FileWriter;

import org.usfirst.frc.team4587.robot.Constants;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class TestPath {
	public Trajectory left;
	public Trajectory right;
	public TestPath(){
		Waypoint[] points = new Waypoint[] {
			    //new Waypoint(-4, -1, Pathfinder.d2r(-45)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
				new Waypoint(0, 0, 0),
			    //new Waypoint(3,0,0)
			    new Waypoint(102.0/12.0, 55.5/12.0, 0),// Waypoint @ x=-2, y=-2, exit angle=0 radians
			    //new Waypoint(0, 0, Pathfinder.d2r(180)),
			    //new Waypoint(5, 5, Pathfinder.d2r(90))                           // Waypoint @ x=0, y=0,   exit angle=0 radians
			};
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, Constants.kStepSizeSeconds, 
				Constants.kMaxFeetPerSecond, Constants.kMaxAcceleration, Constants.kMaxJerk);

		// Generate the trajectory
		Trajectory trajectory = Pathfinder.generate(points, config);
		
		// The distance between the left and right sides of the wheelbase is 0.6m
		double wheelbase_width = Constants.kWheelBaseFeet;

		// Create the Modifier Object
		TankModifier modifier = new TankModifier(trajectory);

		// Generate the Left and Right trajectories using the original trajectory
		// as the centre
		modifier.modify(wheelbase_width);

		left  = modifier.getLeftTrajectory();       // Get the Left Side
		right = modifier.getRightTrajectory();      // Get the Right Side
		try{
		FileWriter w = new FileWriter(new File("/home/lvuser/pathLog.csv") );

	    w.write("dt,x,y,pos,vel,acc,jerk,heading\n");
		for (int i = 0; i < left.length(); i++) {
		    Trajectory.Segment seg = left.get(i);
		    w.write(seg.dt+","+ seg.x+","+ seg.y+","+ seg.position+","+ seg.velocity+","+ 
		            seg.acceleration+","+ seg.jerk+","+ seg.heading+"\n");
			}
		for (int i = 0; i < right.length(); i++) {
		    Trajectory.Segment seg = right.get(i);

		    w.write(seg.dt+","+ seg.x+","+ seg.y+","+ seg.position+","+ seg.velocity+","+ 
		            seg.acceleration+","+ seg.jerk+","+ seg.heading+"\n");
			}
		w.close();
		}catch(Exception e){
			
		}
	}
	
	
}
