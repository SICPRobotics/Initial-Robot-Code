package org.usfirst.frc.team5822.robot;


import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	SICPRobotDrive myRobot; //drive train
	SICPRobotDrive intake; //bag motors at front of ball intake
	CANTalon armR; //this is the arm that rotates the ball intake
	
	Joystick stickx; //xbox
	Joystick stickj; //joystick
	
	int gyroCounter; 
	double speedCountTest; 
	
	Timer autoTimer = new Timer();
	
	//sensors
	ADXRS450_Gyro gyro;
	AnalogInput ultrasonic;
		
	//variables for cameras
    int cameraID = 1; 
	public static USBCamera cameraFront;
/*	public static USBCamera cameraBack;*/  //commented out because this camera's USB broke at CIR
	public static USBCamera activeCamera; 
	Image img =  NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0); 
	CameraServer server;
	SendableChooser chooser; 
	
	int turnCount;
	boolean inverted; 
	int autoStep; 
	
	
	//variables for arm calibration
	boolean isCalibrating = false; 
    double lastPosition; 
    Timer calTimer = new Timer(); 
    
    //variables for gyro turning method
    boolean isTurning = false; 
	boolean	isTurned = false;
	boolean turnBack = false; 
	double holdPosition = 180; 
   
	//variables for our PID
	PIDController gPid; 
	double tPower; 
	
	
	int teleopFunction; 
	

	//encoder values for heights for the ball intake	
	private final int INTAKEHEIGHT = -69000; 
	private final int LOWBARHEIGHT = -62000; 
	private final int LOWBARFORWARDHEIGHT = -60000; 
	private final int SHOOTHEIGHT = -54000; 
	
	//for the sendable chooser
	String defense; 
	 
 	//varaibles for the driveto method with the ultrasonic sensor
 	 double ultraVal; 
     boolean seenOnce;
	
	    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	 public void robotInit() {
    	
		//camera set up 
		//everything with the back camera taken out - back camera USB broke at CIR
		server = CameraServer.getInstance();
        server.setQuality(25);
		cameraFront = new USBCamera("cam1"); //changed from cam0 3-17
/*		cameraBack = new USBCamera("cam1");*/
		cameraFront.openCamera();
/*		cameraBack.openCamera();*/
		cameraFront.startCapture(); // startCapture so that it doesn't try to take a picture before the camera is on
		
		activeCamera = cameraFront; 
       /*the camera name (ex "cam0") can be found through the roborio web interface*/
        server.startAutomaticCapture("cam"+cameraID);  
        
    	//drive train set up and all motors inverted
    	myRobot = new SICPRobotDrive(0, 1, 2, 3);
    	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kFrontLeft, true);
    	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kRearLeft, true);
    	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kFrontRight, true);
    	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kRearRight, true);
    	inverted = true; 
    	
    	//sets up intake
    	intake = new SICPRobotDrive(5, 6);
	
    	//sets up joysticks
    	stickj = new Joystick(0);  
    	stickx = new Joystick(1); 
    	
    	//sets up gyro 
    	gyro = new ADXRS450_Gyro();
    	gyro.calibrate();
    	
    	//sets up ultrasonic sensors 
    	ultrasonic = new AnalogInput(1);
    	
    	//sets up the CANTalon that rotates the ball intake
    	armR = new CANTalon(1); 
    	armR.setFeedbackDevice(FeedbackDevice.QuadEncoder); //Set the feedback device that is hooked up to the talon
       	armR.setPID(0.2, 0.001, 100, 0.00, 360, 12, 0); //Set the PID constants (p, i, d)
    	armR.reverseSensor(true);
    	armR.changeControlMode(TalonControlMode.PercentVbus); //Change control mode of talon, default is PercentVbus (-1.0 to 1.0)

	    //sets up SendableChooser for auto code 	   	
    	chooser = new SendableChooser();
    	chooser.initTable(NetworkTable.getTable("Defense Chooser"));
    	chooser.addDefault("Low Bar", "lowbar");
    	chooser.addObject("Reach Defense", "reach");
    	SmartDashboard.putData("Autonomous Defense Chooser", chooser);
    	
    	//varaibles for the driveto method with the ultrasonic sensor
    	ultraVal = 0; 
        seenOnce = false;
        
        //sets up variables for PID with the gyro 
  	    double pGain = 0.9;
  	    double iGain = 0.00521135; 
  	    double dGain = 38.834084496; 
  	    PIDSource gType = new GyroPIDSource ();
  	    GyroPIDOutput gOutput = new GyroPIDOutput(); 
   	    gType.setPIDSourceType(PIDSourceType.kDisplacement);
  		gPid = new PIDController(pGain, iGain, dGain, gType, new GyroPIDOutput(), 0.001); //the lowest possible period is 0.001
    	gPid.setInputRange(-360, 360);  
  	
    	   

    }//End robotInit
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit()  
    {    	
    
  	   	System.out.println("We have been through autonomousInit");
  	 	gyro.reset();
  	    System.out.println("We have reset gyro"); 
    	
  		autoStep = 0; // 
  		
  		//chooses which auto to run
  		defense = chooser.getSelected().toString();
  		if (defense.equals("lowbar")) {
  			System.out.println("RUNNING LOW BAR");
  		} else if (defense.equals("reach")){
  			System.out.println("RUNNING REACH");
  		}
  		
  		
  		startCalibration(); //starts to bring up the ball intake arm	
    }
  	   	   	
    
  

    /**
     * This function is called periodically during autonomous
     */
    
    boolean next = false; 
    boolean first = true; 
    int counter =0; 
    
    public void autonomousPeriodic() 
    {
		if (counter++%1 ==0 )
			System.out.println("Ultrasonic: " + (inchFromHRLV(ultrasonic.getVoltage())) + "\tGyro: " + gyro.getAngle());
		
		double ultraDistance; 
		
		//calibrates the ball intake arm if it is the first time through autonomousPeriodic
    	 if (first)
    	 {	 startCalibration(); 
    	 	first = false; 
    	 	return; 
    	 }  
    	
    	 //calibration 
    	if (isCalibrating)
    	{
    		checkCalibrationStatus(); 
    		return; 
    	}
    	
      	    	
    	if (defense.equals("lowbar")) 
    	{
    		switch (autoStep)
    		{
		    	case 0: //start the PID to go straight
				{
				   	tPower = 0.3;
					gPid.setSetpoint(0);
					gPid.enable();
					autoStep = 1; 
					System.out.println("going to 1");
					adjustArmHeight(-58000); 
					break; 
				}
		    	
		    	case 1:
    			{
	    			ultraDistance = inchFromHRLV(ultrasonic.getVoltage()); //gets sonar reading
	    			adjustArmHeight(-58000); //puts arm in the right place
	    			
	    			if (ultraDistance < 24) //sensor has seen the low bar flaps
	    			{
	   					autoStep = 2; 
	   					System.out.println("going to 2");
	   					//dont think this reset does anything
	   					autoTimer.reset();
	    			}
	    			break;
	    		}
    			
		    	case 2:
		    	{ 
		    		ultraDistance = inchFromHRLV(ultrasonic.getAverageVoltage()); //gets sonar reading
		    		//150 may be bigger than necessary - are there 12.5feet to wall after low bar?
		    		if (ultraDistance > 150) 
		    		{
		    			autoTimer.reset();
		    			autoTimer.start();
		    			System.out.println("going to 3");
		    			autoStep = 3; 
		    		}
		    		
		    		break;
		    		   		
		    	}
		    	
		    	case 3: 
		    	{
		    		ultraDistance = inchFromHRLV(ultrasonic.getVoltage()); 
		    		//is this long enough?
		    		if (autoTimer.get() > 0.3)
		    		{
		    			if (ultraDistance > 100) //robot has seen the wall on the other side
		    			{
		    				autoTimer.reset();
		    				System.out.println("going to 4");
		    				autoStep = 4; 
		    			}
		    			
		    			else 
		    			{
		    				autoStep = 2; //back to looking for the wall 
		    				System.out.println("going to 2"); //sensor did not see the wall. This is here incase the robot front tilts up and the sensor sees the ceiling
		   
		    			}
		    			
		    		}
		    		break;
		    	}
		    	
		   	    
				case 4: // waits until the ultrasonic reads the right distance
	    		{
	    			ultraDistance = inchFromHRLV(ultrasonic.getVoltage()); 
	    		
	    			if (ultraDistance < 106) //used to be 62
	    			{
	    				gPid.reset(); //changed to reset
	    				myRobot.drive(0,  0); //added the -0.01 power
	    				autoStep = 30; 
	    				tPower = 0.175; //slows the robot down so it won't over shoot as much
	    				gPid.enable();
	    				System.out.println(ultraDistance); 
	    				System.out.println("going to 5");
	    			}
	    			
	    			break;
	    		}
	    		
				case 30: 
				{
					ultraDistance = inchFromHRLV(ultrasonic.getVoltage());
					
					//this distance still not correct - reason we missed the goal at CIR
					if (ultraDistance < 94) //changed by adding 33
					{
						gPid.reset();
						myRobot.drive(0, 0);
						autoStep = 5; 						
					}
					break;
				}
		    
				case 5: //uses a PID to go backwards
	    	    {
	    	    	tPower = -0.175; //sets a negative power
	    	    	gPid.enable();
	    	    	System.out.println("going to 6");
	    	    	autoStep = 6;  
	    	    	break;
	    	    }
		    
				case 6: // goes backwards until back at the ultrasonic threshold 
	    	    {
	    	    	ultraDistance = inchFromHRLV(ultrasonic.getVoltage()); 
	    	    	
	    	    	//this distance still not correct - reason we missed the goal at CIR
	    	    	if (ultraDistance >= 94)
	    	    	{
	    	    		System.out.println(ultraDistance);
	    	    		gPid.disable();
	    	    		myRobot.setLeftRightMotorOutputs(0,0);
	    	    		autoStep = 7; 
	    	    		System.out.println("going to 7");
	    	    	}
	    	    	break;
	    	    }
	    	    
				case 7: 
	    		{
	    			adjustArmHeight(SHOOTHEIGHT); //intake put to the right position to shoot
	    			
	    			if (gyro.getAngle()< 58) //this angle looked good at CIR
	    				myRobot.setLeftRightMotorOutputs(-0.3,0.3);
	    			
	    			else 
	    			{
	    				autoStep = 8; 
	    				System.out.println("going to 8");
	    				myRobot.setLeftRightMotorOutputs(0,0);
	    			}
	    			
	    			break;
	    		}
			
				case 8: // turns slowly back in case of overshoot 
	    		{
	    			if (gyro.getAngle() >= 58) 
	    				myRobot.setLeftRightMotorOutputs(0.17,-0.17);
	    			else 
	    			{
	    				autoStep = 9; 
	    				System.out.println("going to 9");
	    			}
	    			
	    			break;
	    		}
			
				case 9: //uses a PID to start to go forward
	    		{
	    			myRobot.drive(0, 0);
	    			gPid.setSetpoint(55); //test this angle
	    			tPower = 0.25; 
	    			gPid.enable();
	    			autoStep=10; 
	    			System.out.println("going to 10");
	    			
	    			break;
	    		}
	    		
				case 10: //starts timer
				{
					
					autoTimer.reset();
					autoTimer.start();
					autoStep = 11;
					System.out.println("going to 11");
						
					break;
				}
					
				case 11: //drive forward for time
				{
									
					if (autoTimer.get() > 2) //test this num
					{
						gPid.disable();
						myRobot.setLeftRightMotorOutputs(0,0);
						System.out.println("going to 12");
						autoStep = 12; 
					}
					
					break;
				}
				
				case 12: //shoot
				{
					//intake commented out for CIR elims so we wouldn't shoot the ball
					/*intake.setLeftRightMotorOutputs(-1, -1);*/				
					
					if (autoTimer.get()>3)  
					{
						autoStep = 13;
						System.out.println("going to 13");
						
					}
					
					break;
				}
				
				case 13: 
				{
					myRobot.setLeftRightMotorOutputs(0,0);
				/*	intake.drive(0, 0);*/
					gPid.reset(); //added 3-3
					autoStep = 14; 
					break; 
				}
				
				//added 3-3
				case 14: //stops everything from moving
				{
					myRobot.setLeftRightMotorOutputs(0, 0);
					/*intake.drive(0, 0);*/
					break;
				}
			    		
	    	}
    	}
       	
    	    		
    	if (defense.equals("reach"))
    	{
    		switch(autoStep)
    		{ 
	    		case 0: //start the PID to go straight
				{
				   	tPower = 0.3; //set to higher power
					gPid.setSetpoint(0);
					gPid.enable();
					autoStep = 1; 
					System.out.println("going to 1");
					autoTimer.reset();
   					autoTimer.start();
					break; 
				}
		    	
		    	case 1: //approaches defense quickly
				{
	     			
	    			if (autoTimer.get()>0.75) //test this number
	    			{
	   					autoStep = 2; 
	   					System.out.println("going to 2");
	   					
	    			}
	    			break;
	    		}
				
		    	case 2: //continues to drive forward slowly into the defense
		    	{
		    		tPower = 0.175; 			
		    	}
			
    		}
    			
    	}
    }
   
	
    
    /*
     * This function is called once each time the robot enters tele-operated mode*/
        
   
    public void teleopInit()
    {
   
    	if (gPid != null && gPid.isEnabled()) //make sure gPid is disabled 
    	 	 gPid.disable(); 
    	
    	myRobot.drive(0, 0);
    	myRobot.setSafetyEnabled(false);
    	autoStep=0; //auto sequence counter reset 
    }


    //enum for ball intake heights
    public enum TeleopFunctions 
    {
    	NONE(0), LOWBARBACKWARD(1), GRABBALL(2), SHOOT(4), RESET(3), HOLD(4), LOWBARFORWARDS(5); 
    	
    	public final int val; 
    	
    	private TeleopFunctions(int value){
    		val = value; 
    	}
    }
    
    
    
    double voltage; 
    double value; 
    
    boolean isCalibrated = false; 
    boolean firstTime = true;
    double moveValue=0; 
    double rotateValue = 0; 
    int invertCount=1; 
    double voltage2=0; 
    
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() 
    {
    	//this line was sometimes was uncommented in a match but should always be commented out unless troubleshooting
    	System.out.println("UltrasonicHRLV: " + (inchFromHRLV(ultrasonic.getVoltage())) + "  Gyro: " + gyro.getAngle());
    	
    	//calibrates the arm (raises it up until the hard stop) 
    	if (isCalibrating)
    		checkCalibrationStatus(); 
    	
    	//for turning 180 degrees
    	if (isTurning)
    	{
    		checkTurningStatus(); 
    		return; 
    	}
  	   	
    	TeleopFunctions chosen; 
    	int currentPosition = armR.getEncPosition();
    	
    	//this line was also uncommented at CIR - should be commented out 
    	System.out.println(currentPosition);
    	
    	//get the value of the slider on the joystick 
    	//this is used so the driver can adjust max speed anywhere from 60% to 100% 
    	double scale = stickj.getRawAxis(3)*-1; 
    	scale = ((scale+1)/5)+0.6; 

    	//gets the x and y axis values from the joystick 
    	moveValue = stickj.getRawAxis(1);
    	rotateValue = stickj.getRawAxis(0); 
    	
    	//dead zone on y axis value
    	if (Math.abs(moveValue)<0.005)
    		moveValue = 0; 
    	
    	//creates a dead zone on x axis value only if the y axis value is small 
    	if (Math.abs(rotateValue)<0.005 && Math.abs(moveValue)<0.1)
    		rotateValue = 0;
    	
    	//scale down the values 
    	moveValue = moveValue*scale; 
    	rotateValue = rotateValue*scale; 
    	
    	//if driver tries to turn the robot, stop running the 180 degree turn method
    	if (rotateValue > 0)
    		isTurning = false; 
    	
    	
    	if (!isTurning) //makes sure the driver isn't trying to use the 180 degree turn method
    		myRobot.arcadeDrive(moveValue, rotateValue, true); 
    	
    	//high dead zone for the bag motors - that button on the xbox doesn't always go back to 0 position easily 
    	double intakeAxis = stickx.getRawAxis(5); 
    	if(Math.abs(intakeAxis)<0.25) 
    		intakeAxis=0; 
    	
    	//spins the bag motors
    	intake.drive(intakeAxis, 0);
    	
    	//this code was used so driver could switch between front and back camera 
    	//back camera USB broke at CIR so commented out 
    	
    	/*if(stickj.getRawButton(1))
    	{
    		String camR; 
    		
    		if (cameraID==1)
    		{
    			cameraID=0;
				cameraBack.stopCapture();
				cameraFront.startCapture();
    			activeCamera = cameraFront; 
    		}
    		else
    		{
				cameraFront.stopCapture();
				cameraBack.startCapture();
				activeCamera = cameraBack; 
    			cameraID=1; 
    		}
    		
    		camR = "cam" + cameraID;
    		 
    		
    		while(stickx.getRawButton(5));
    	}*/
    	
    	//starts the 180 degree turn
    	if (!isTurning)
    	{
	    	if(stickj.getRawButton(2))
	    		startTurning();
    	}
    	
    	//provides the driver another way to get out of 180 degree turn method
    	if (stickj.getRawButton(5)) // see if Jack likes this button 
    		isTurning = false; 
    	
		//camera displayed on dahsboard
    	activeCamera.getImage(img);
		server.setImage(img); // puts image on the dashboard
			
		//allows the drive to invert the motors to help with driving backwards
       	if (stickj.getRawButton(3)) //changed 3.14 from button 2 to button 3
       	{  	     		
       		
        	inverted=!inverted; 
        	
        	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kFrontLeft, inverted);
        	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kRearLeft, inverted);
        	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kFrontRight, inverted);
        	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kRearRight, inverted);
    
       	}
       	
       	//button functions on the xBox 
        //The buttons on the xBox are Y(top, 3) B(right,2) A(bottom, 1) X(left, 4)    
      	chosen = TeleopFunctions.NONE;     	
    	
        //Y is for getting the arm to the right place to shoot the ball
        if (stickx.getRawButton(3)==true)
        	chosen = TeleopFunctions.SHOOT; 
        
        //A is for getting the arm to the right place for crossing low bar when ball intake is in backwards
        else if (stickx.getRawButton(1)==true)
        	chosen = TeleopFunctions.LOWBARBACKWARD; 
        
        //Upper Left button is for bringing the intake back up to the hard stop (calibrating it) 
        else if (stickx.getRawButton(5)==true)
            chosen = TeleopFunctions.RESET;
           
        //B is for getting the arm to the right place to grab a ball  
        else if (stickx.getRawButton(2)==true)
        	chosen = TeleopFunctions.GRABBALL;
        
        //Upper Right Button is for holding the intake arm in its place
        else if (stickx.getRawButton(6)==true)
        	chosen = TeleopFunctions.HOLD;
        
        //X is for getting the arm to the right place for crossing low bar when ball intake is in front
        else if (stickx.getRawButton(4)== true)
        	chosen = TeleopFunctions.LOWBARFORWARDS; 
        
        //calls the method adjustArmHeight based on which button is pressed   
        switch (chosen)
        {
	        case NONE: 
	        	break; 
	        
	        case RESET:
	        	startCalibration(); 
	        	break; 
	        	
	        case GRABBALL:
	        	adjustArmHeight(INTAKEHEIGHT); 
	        	break;
	        	
	        case LOWBARBACKWARD: 
	        	adjustArmHeight(LOWBARHEIGHT); 
	        	break; 
	        	
	        case SHOOT:
	        	adjustArmHeight(SHOOTHEIGHT); 
	        	break; 
	        	
	        case HOLD: 
	        	adjustArmHeight(armR.getPosition()); 
	        	break; 
	        	
	        case LOWBARFORWARDS: 
	        	adjustArmHeight(LOWBARFORWARDHEIGHT);
        }
        
        //allows arm to also be manually controlled 
        if(!isCalibrating&&chosen!=TeleopFunctions.HOLD) {
    		double armAxis = stickx.getRawAxis(1); 

    		if(armR.getControlMode()==TalonControlMode.Position)
    		{
    			System.out.println("EXITING POSITION MODE");
    			
    			//sets dead zone on the arm
    			if (Math.abs(armAxis)>0.2)
    			{
    				armR.changeControlMode(TalonControlMode.PercentVbus);
    				armR.set(armAxis*0.5);
    			}
    		}

    		else 
    			armR.set(armAxis*0.5);
    	}
        		
           
    }
    	
  	
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
		System.out.println("UltrasonicHRLV: " + (inchFromHRLV(ultrasonic.getVoltage()))+ "\tGyro: " + gyro.getAngle());
    } //End Test Periodic 
    
    public void startCalibration()
    {
    	//sets up all variables for calibrating
    	isCalibrating=true; 
    	isCalibrated = false; 
    	armR.changeControlMode(TalonControlMode.PercentVbus);
    	lastPosition = armR.getPosition();
    	calTimer.start();
    	armR.set(0.35);
    	
    }
    
    public boolean checkCalibrationStatus()
    {
    	double timerVal = calTimer.get();
/*    	System.out.println("calTimer:" + timerVal);*/
    	
    	//calibration will run for at least 0.1 seconds
    	if (timerVal<0.1)
    		return false; 
    	
		double newPosition = armR.getPosition(); 
		/*System.out.println("cDelta:" + (newPosition-lastPosition));*/
	
		
		//stops calibration if the current encoder position is different from the previous by less than 100
		//stops calibration is it has been running for over 10 seconds 
		if (Math.abs(newPosition-lastPosition)<100 || calTimer.get()>10) 
		{
			calTimer.stop();
			calTimer.reset();
			
			armR.setPosition(0); 
			armR.set(0);
			System.out.println("DONE Calibrating"); 
			isCalibrating = false; 
			isCalibrated = true; 
			armR.enableBrakeMode(true);
			return true; 
			
		}
		
		lastPosition = newPosition; 
	
    	return false;  
    } 
    
    //sets up all variables for the 180 degree turn
    public void startTurning()
    {
    	gyro.reset();
    	isTurning=true; 
    	isTurned = false; 
    	turnBack = false; 
    	holdPosition = gyro.getAngle()+180; //theoretically should be 180 because gyro just reset
    	myRobot.setLeftRightMotorOutputs(-0.5, 0.5);
    	
    }
    
 
    
    public boolean checkTurningStatus()
    {
    	    	
		double newPosition = gyro.getAngle(); 
		
		if (turnBack)
		{
			myRobot.setLeftRightMotorOutputs(0.25, -0.25); //turns robot back to hold position slowly 
			if (holdPosition > newPosition)
			{
				isTurning = false; 
				isTurned = true; 
				turnBack = false; 
				return true; 
			}
		}
		
		else 
		{
			if (newPosition < 120)
				myRobot.setLeftRightMotorOutputs(-0.5, 0.5);
			
			else  
				myRobot.setLeftRightMotorOutputs(-0.35, 0.35); //slows robot down when it gets closer
			
			if (newPosition > holdPosition) //sees if robot has passed the holdPosition 
			{
				turnBack = true; 
				
			}
		}
		
    	return false;  
    } 

       
    
    public void adjustArmHeight (double height)
    {
    	//makes sure the arm has been calibrated at least once during the match 
    	if (isCalibrated)
    	{
	    	armR.changeControlMode(TalonControlMode.Position); //Change control mode of talon, default is PercentVbus (-1.0 to 1.0)
	    	armR.setFeedbackDevice(FeedbackDevice.QuadEncoder); //Set the feedback device that is hooked up to the talon
	    
	    	armR.set(height); 
	    	armR.reverseSensor(true); 
	    	armR.enableControl(); //Enable PID control on the talon
    	}
    	
    	else 
    		System.out.println("YOU NEED TO CALIBRATE!!"); 	
        }
    
    //method to get the distance in inches from ultrasonic sensor
    public double inchFromHRLV (double volts)
    {
    	double v = (volts*43.796)-2.6048; //equation determined graphically after running some tests
    	return v; 
    }
    
 
  //internal class to write to myRobot (a RobotDrive object) using a PIDOutput
    public class GyroPIDOutput implements PIDOutput 
    {
    	int counter = 0; 
    	   	
    	//this never used in the code
    	//elected to make tPower a global variable instead 
    	public void setPower(double power)
    	{
    		tPower = power; 
    	}
    	
	    public void pidWrite(double output) 
	    {
	    	
	    	double scaled = output*0.1;
	    	
	    	if(gPid.isEnabled())//this was added 3-3
	    		//scaled is used to change the power outputs to both sides 
	    		myRobot.setLeftRightMotorOutputs(-1*(tPower+scaled), -1*(tPower-scaled));
	    	
	    	/*System.out.println("Left wheel: " + (-(tPower+scaled)));
	    	System.out.println("Right wheel: " + (-(tPower-scaled)));*/
	    	
	    		    
	    }
    }
    
    //sets up a PIDSource to work with the gyro 
    public class GyroPIDSource implements PIDSource
    {
    	  /**
    	   * Set which parameter of the device you are using as a process control
    	   * variable.
    	   *
    	   * @param pidSource
    	   *            An enum to select the parameter.
    	   */
    	
    	private PIDSourceType myType; 
    	
    	  public void setPIDSourceType(PIDSourceType pidSource)
    	  {
       		  myType = pidSource; 
    	  }
    	  /**
    	   * Get which parameter of the device you are using as a process control
    	   * variable.
    	   *
    	   * @return the currently selected PID source parameter
    	   */
    	  public PIDSourceType getPIDSourceType()
    	  {
    		  return myType; 
    	  }
    	  /**
    	   * Get the result to use in PIDController
    	   *$
    	   * @return the result to use in PIDController
    	   */
    	  public double pidGet()
    	  {
    		  /*System.out.println(gyro.getAngle());*/
    		  return gyro.getAngle();	  
    	  }
    	}
    
      
     //method to read values from our LV sensor which broke week 6
      public double inchFromLV (double volts)
      {
    	 double v = (volts*91.019) + 5.0692; 
    	 return v; 
      }
    

     //the next two methods were methods we worked on to get the robot to the right place using the ultrasonic sensor
     //we never finished these methods and do not use them in the code 
      
    //returns 0 if it has not gotten to the right distnace
    //returns 1 if it has seen it once and needs to go backwards
    //returns 2 if it is done doing its thing
    public int ultraGoTo (double distance, AnalogInput ultra, boolean HRLV)
    { 
    	 	    	    	
    	if (HRLV)
    		ultraVal = inchFromHRLV(ultra.getVoltage()); 
    	else 
    		ultraVal = inchFromLV(ultra.getVoltage()); 
    	
    	System.out.println("Current Ultra View: " + ultraVal);
    	
    	if (!seenOnce)
    	{ 
	    	if (distance < ultraVal)
	    	{
	    		return 0; 
	    	}
	    	
	    	else if (distance > ultraVal)
	    	{
	    		seenOnce = true; 
	    		return 1;  
	    	}
    	}
    	
    	else 
    	{
    		if (distance >= ultraVal)
    		{
    			return 1; 
    		}
    		
    		else 
    		{
    			seenOnce = false; 
    			return 2; 
    		}
    		
    	}
   
    	return 0; 
    }
    
    /*public boolean ultraGoTo (double distance, AnalogInput ultra, boolean HRLV, double speedTo, double speedBack)
    { 
    	boolean toSpot = false; 
    	
    	    	
    	if (HRLV)
    		ultraVal = inchFromHRLV(ultra.getVoltage()); 
    	else 
    		ultraVal = inchFromLV(ultra.getVoltage()); 
    	
    	System.out.println("Current Ultra View: " + ultraVal);
    	
    	if (!seenOnce)
    	{
    		toSpot = false; 
	    	if (distance < ultraVal)
	    	{
	    	
	    	}
	    	
	    	else if (distance > ultraVal)
	    	{
	    		seenOnce = true; 
	    	}
    	}
    	
    	else 
    	{
    		if (distance >= ultraVal)
    		{
    			myRobot.drive(-speedBack, 0);
    		}
    		
    		else 
    		{
    			myRobot.drive(0, 0);
    			toSpot = true; 
    			seenOnce = false; 
    		}
    		
    	}
   
    	return toSpot; 
    }
    */    
    
}
