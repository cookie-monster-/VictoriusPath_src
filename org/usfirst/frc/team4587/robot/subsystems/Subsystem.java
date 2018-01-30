package org.usfirst.frc.team4587.robot.subsystems;

import org.usfirst.frc.team4587.robot.loops.Looper;

public abstract class Subsystem {
    public void writeToLog() {
    };

    public abstract void outputToSmartDashboard();

    public abstract void stop();

    public abstract void zeroSensors();

    public abstract void registerEnabledLoops(Looper enabledLooper);
}