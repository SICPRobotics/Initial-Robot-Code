package org.usfirst.frc.team5822.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SICPRobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	SICPRobotDrive myRobot;
	SICPRobotDrive autoRobot;
	Joystick stick;
	int autoLoopCounter;
	Gyro gyro = new AnalogGyro(1);
	double Kp = 0.03; 
			
	
	
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	myRobot = new SICPRobotDrive(0, 1, 2, 3);
    	stick = new Joystick(0);
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() 
    {
    	
    	autoLoopCounter = 0;
    	
    	
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	/*if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			myRobot.drive(-0.5, 0.0); 	// drive forwards half speed
			autoLoopCounter++;
			} else {
			myRobot.drive(0.0, 0.0); 	// stop robot
		}*/
    	
    	
    	
    	//this is test code for the gyro. if it works, 
    	//it should make the robot go forward straight in autonomous 
    	//added by Greta Rauch 1-26 
    
    	autoRobot = new SICPRobotDrive(0, 1, 2, 3); 
    	gyro.reset();
    	
    	if (autoLoopCounter==0)
    	{
    		while(gyro.getAngle()<90)
    		{
    			
    		}
    	}
    	
    	while(isAutonomous()&&isEnabled())
    	{
    		double angle = gyro.getAngle(); //get current heading
    		autoRobot.arcadeDrive(0.4, angle*Kp);
    		Timer.delay(0.004);
    		autoLoopCounter ++; 
    	}
    	
    	autoRobot.drive(0.0, 0.0);
    	
    	while (isAutonomous()&&isEnabled())
    	{
    		autoRobot.drive(0.15, 0.0);
    	}
    	
    	
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        myRobot.arcadeDrive(stick);
        //TEST DOMINIC
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
