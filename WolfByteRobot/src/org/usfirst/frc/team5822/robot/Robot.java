package org.usfirst.frc.team5822.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Servo;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	SICPRobotDrive myRobot;
	Joystick stick;
	int autoLoopCounter;
	double speedCountTest; 
	Gyro gyro = new AnalogGyro(1);
	double Kp = 0.03; 
	boolean buttonPressedA;
    boolean buttonPressedB;
	JoystickButton motorButtonA;		
	JoystickButton motorButtonB;	
	SICPRobotDrive intake;
	Servo servo1;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */

    public void robotInit() {
    	motorButtonA = new JoystickButton(stick, 1);
    	motorButtonB = new JoystickButton(stick, 2);
    	myRobot = new SICPRobotDrive(0, 1, 2, 3);
    	intake = new SICPRobotDrive(5, 6);
    	stick = new Joystick(0);  
    	servo1= new Servo(7);
    }//End robotInit
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit()  {    	
    	autoLoopCounter = 0;
  	
    	System.out.println("We have been through autonomousInit");
    	
    	
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
    	//this has yet to be run
    
    	    /*	gyro.reset();
    	
    	if (autoLoopCounter==0) {
    		while(gyro.getAngle()<90) {
    			
    		}//End while gyro
    	}//End autoLoopCounter
    	
    	while(isAutonomous()&&isEnabled()) {
    		double angle = gyro.getAngle(); //get current heading
    		autoRobot.arcadeDrive(0.4, angle*Kp);
    		Timer.delay(0.004);
    		autoLoopCounter ++; 
    	}//End While isAutonomous 
    	
    	autoRobot.drive(0.0, 0.0);*/
    	
   	   	
    	//this code below speeds the robot up and works! 
    	
    	System.out.println("autonomousPeriodic; " + speedCountTest);
    	
    	myRobot.drive(speedCountTest, 0.0);
    	
    	Timer.delay(0.004);
    	speedCountTest += 0.001; 
    	
    	if (speedCountTest>0.5)
    		speedCountTest = 0; 
    	
    	

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
                
   		while(isEnabled()){
   			//buttonPressedA = motorButtonA.get();
   	       // buttonPressedB = motorButtonB.get();
        	double currentSpeed = 0;


    	
    		/*
    		//If right bumper is pressed, add.2
    		if ()
    			currentSpeed += .2;
    		
    		//If left bumper is pressed, subtract .2
    		if ()
    			currentSpeed -= .2;
    		
    		
    		//If you press A, motor is ready to run
    		if (){
    			motor.set(currentSpeed);
    		}*/
   			
   			
   			
   			
   			if (stick.getRawButton(1) == true){
   			intake.setInvertedMotor(SICPRobotDrive.MotorType.kRearLeft, true);
   			intake.setInvertedMotor(SICPRobotDrive.MotorType.kRearRight, true);
   				intake.arcadeDrive(0.7, 0);}
   			if (stick.getRawButton(2) == true){
   				//intake.setInvertedMotor(SICPRobotDrive.MotorType.kRearLeft, true);
   				intake.setInvertedMotor(SICPRobotDrive.MotorType.kRearRight, true);
   				intake.arcadeDrive(0.7, 0);}
   			if (stick.getRawButton(3)== true)
   			{
   				servo1.setAngle(90);
   			}
   			if(stick.getRawButton(3)==false)
				{
				servo1.setAngle(0);
				}
   			}//End if Button Pressed  		
   		}//End While isEnabled
 
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    } //End Test Periodic 
    
}
