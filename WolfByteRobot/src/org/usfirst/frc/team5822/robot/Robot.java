package org.usfirst.frc.team5822.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	//Haffner comment
	RobotDrive myRobot; 
	
    JoystickButton motorbutton;
    JoystickButton motorbutton2; 
	Joystick stick; 
	int autoLoopCounter;
	
	VictorSP motor; //any extra single motor 
	
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	myRobot = new RobotDrive(0, 1, 2, 3);
    	stick = new Joystick(0);
    	
    	motorbutton = new JoystickButton(stick, 1); //this controls the A button
    	motorbutton2 = new JoystickButton (stick, 2); //this controls the B button
        motor = new VictorSP(5); //this goes to the fifth PWN channel
    	
    	
    	
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	autoLoopCounter = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			myRobot.drive(-0.5, 0.0); 	// drive forwards half speed
			autoLoopCounter++;
			} else {
			myRobot.drive(0.0, 0.0); 	// stop robot
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
    
    
    	for (int x = 0; x < 10; x++)
    	{
      
  
    		boolean yesorno = motorbutton.get();
    		
    		if (yesorno == true)
    		{
    		motor.set(0.3);
    		}
    
    		
   
    
	    x--;
	    
	    boolean yes2 = motorbutton2.get(); 
	    
	    if (yes2)
	    		motor.set(-0.3);
	    if (yesorno == false && yes2 ==false)
	    	motor.set(0);
   
    	}
    	
    
    	}
    
    	
    	/**;
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
