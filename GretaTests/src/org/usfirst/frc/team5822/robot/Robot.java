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
	SICPRobotDrive myRobot;
	SICPRobotDrive intake;
	Joystick stickx;
	Joystick stickj; 
	int gyroCounter; 
	double speedCountTest; 
	double Kp = 0.03; 
	Timer autoTimer = new Timer();
	ADXRS450_Gyro gyro;
	AnalogInput ultrasonic;
	AnalogInput ultrasonic2; 
    public int timesLoop = 0; 
    int cameraID = 0; 
	public static USBCamera cameraFront;
	public static USBCamera cameraBack;
	public static USBCamera activeCamera; 
	
	int turnCount;
	
	boolean inverted; 
	
	int autoStep; 

	boolean isCalibrating = false; 
    double lastPosition; 
    Timer calTimer = new Timer(); 
   
    
	double tPower; 
	int teleopFunction; 
	
	CANTalon armR; //this is the arm that rotates the ball intake
	
	Timer teleTimer = new Timer();
	
	PIDController gPid; 
	
	private final int INTAKEHEIGHT = -69000; 
	private final int LOWBARHEIGHT = -62000; 
	private final int SHOOTHEIGHT = -54000; 
	Image img =  NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0); 

	
	 CameraServer server;
	 SendableChooser chooser; 

	 String defense; 
	 
 	//varaibles for the driveto method with the ultrasonic sensor
 	
     double ultraVal; 
     boolean seenOnce;
	
	    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	 public void robotInit() {
    	
		server = CameraServer.getInstance();
        server.setQuality(25);
		cameraFront = new USBCamera("cam0");
		cameraBack = new USBCamera("cam1");
		cameraFront.openCamera();
		cameraBack.openCamera();
		cameraFront.startCapture(); // startCapture so that it doesn't try to take a picture before the camera is on
		
		activeCamera = cameraFront; 
       /*the camera name (ex "cam0") can be found through the roborio web interface*/
        server.startAutomaticCapture("cam"+cameraID);  
        
    	//all motors inverted
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
    	
   	
    	gyro = new ADXRS450_Gyro();
    	gyro.calibrate();
    	
    	ultrasonic = new AnalogInput(1);
    	ultrasonic2 = new AnalogInput(0); 
    	
    	armR = new CANTalon(1); 
    	
    	armR.setFeedbackDevice(FeedbackDevice.QuadEncoder); //Set the feedback device that is hooked up to the talon
    
    	armR.setPID(0.2, 0.001, 100, 0.00, 360, 12, 0); //Set the PID constants (p, i, d)
    	armR.reverseSensor(true);

    	armR.changeControlMode(TalonControlMode.PercentVbus); //Change control mode of talon, default is PercentVbus (-1.0 to 1.0)

	    	   	
    	chooser = new SendableChooser();
    	chooser.initTable(NetworkTable.getTable("Defense Chooser"));
    	chooser.addDefault("Low Bar", "lowbar");
    	chooser.addObject("Ramparts", "ramparts");
    	chooser.addObject("Moat", "moat");
    	chooser.addObject("Rough Terrain", "rough");
    	chooser.addObject("Rock Wall", "rockwall");
    	chooser.addObject("Spy Bot", "spy");
    	chooser.addObject("Reach Defense", "reach");

    	SmartDashboard.putData("Autonomous Defense Chooser", chooser);
    	
    	//varaibles for the driveto method with the ultrasonic sensor
    	
    	ultraVal = 0; 
        seenOnce = false;
        
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

    	
    	timesLoop=0; 
  		autoStep = 0; 
  		
  		defense = chooser.getSelected().toString();
 		
   
  		if (defense.equals("lowbar")) {
  			System.out.println("RUNNING LOW BAR");
  		} else if (defense.equals("ramparts")) {
  			System.out.println("RUNNING RAMPARTS");
  		} else if (defense.equals("moat")) {
  			System.out.println("RUNNING MOAT");
  		} else if (defense.equals("rough")) {
  			System.out.println("RUNNING ROUGH TERRAIN");
  		} else if (defense.equals("rockwall")){
  			System.out.println("RUNNING ROCKWALL");
  		} else if (defense.equals("spy")){
  				System.out.println("RUNNING SPY"); 
  		} else if (defense.equals("reach")){
  			System.out.println("RUNNING REACH");
  		}
  		
  		
  		startCalibration(); 	
    }
  	   	   	
    
  

    /**
     * This function is called periodically during autonomous
     */
    
    boolean next = false; 
    boolean first = true; 
    int counter =0; 
    
    public void autonomousPeriodic() 
    {
		if (counter++%20 ==0 )
			System.out.println("Ultrasonic: " + (inchFromHRLV(ultrasonic.getVoltage())) + "\tGyro: " + gyro.getAngle());
		
		double ultraDistance; 
    	 if (first)
    	 {	 startCalibration(); 
    	 	first = false; 
    	 	return; 
    	 }  
    	
    	if (isCalibrating)
    	{
    		checkCalibrationStatus(); 
    		return; 
    	}
    	
    	/*if (autoStep ==0)
    	    if (ultraGoTo (24, ultrasonic, true, -0.175, -0.12))
    	    	autoStep = 1; 
    	*/
    	
    	
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
					adjustArmHeight(-60000); 
					break; 
				}
		    	
		    	case 1:
    			{
	    			ultraDistance = inchFromHRLV(ultrasonic.getVoltage()); //use sonar to figure out when there
	    			adjustArmHeight(-60000);
	    			
	    			if (ultraDistance < 24) //test this number
	    			{
	   					autoStep = 2; 
	   					System.out.println("going to 2");
	   					autoTimer.reset();
	    			}
	    			break;
	    		}
    			
		    	case 2:
		    	{ 
		    		ultraDistance = inchFromHRLV(ultrasonic.getAverageVoltage()); 
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
		    		if (autoTimer.get() > 0.3)
		    		{
		    			if (ultraDistance > 100)
		    			{
		    				autoTimer.reset();
		    				System.out.println("going to 4");
		    				autoStep = 4; 
		    			}
		    			
		    			else 
		    			{
		    				autoStep = 2;
		    				System.out.println("going to 2");
		    			}
		    			
		    		}
		    	}
		    	
		   	    
				case 4: // waits until the ultrasonic reads the right distance
	    		{
	    			ultraDistance = inchFromHRLV(ultrasonic.getVoltage()); 
	    			
	    			if (ultraDistance < 68)
	    			{
	    				gPid.disable();
	    				myRobot.drive(0,  0);
	    				autoStep = 5; 
	    				System.out.println(ultraDistance); 
	    				System.out.println("going to 5");
	    			}
	    			
	    			break;
	    		}
		    
				case 5: //uses a PID to go backwards
	    	    {
	    	    	tPower = -0.175; 
	    	    	gPid.enable();
	    	    	System.out.println("going to 6");
	    	    	autoStep = 6;  
	    	    	break;
	    	    }
		    
				case 6: // goes backwards until back at the ultrasonic threshold 
	    	    {
	    	    	ultraDistance = inchFromHRLV(ultrasonic.getVoltage()); 
	    	    	
	    	    	if (ultraDistance >= 68)
	    	    	{
	    	    		System.out.println(ultraDistance);
	    	    		gPid.disable();
	    	    		autoStep = 7; 
	    	    		System.out.println("going to 7");
	    	    	}
	    	    	break;
	    	    }
	    	    
				case 7: 
	    		{
	    			adjustArmHeight(SHOOTHEIGHT); 
	    			if (gyro.getAngle()< 58.1) //test this angle
	    				myRobot.setLeftRightMotorOutputs(-0.2,0.2);
	    			else 
	    			{
	    				autoStep = 8; 
	    				System.out.println("going to 8");
	    			}
	    			
	    			break;
	    		}
			
				case 8: // turns slowly back 
	    		{
	    			if (gyro.getAngle() >= 58.1) //test this angle
	    				myRobot.setLeftRightMotorOutputs(0.2,-0.2);
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
	    			gPid.setSetpoint(58.1); //test this angle
	    			tPower = 0.25; 
	    			gPid.enable();
	    			autoStep=10; 
	    			System.out.println("going to 10");
	    			
	    			break;
	    		}
	    		
				case 10: //stop at the goal, not exactly sure how this will know when to stop 
				{
					//if (at the distance)
					
					autoTimer.reset();
					autoTimer.start();
					autoStep = 11;
					System.out.println("going to 11");
						
					break;
				}
					
				case 11: //shoot
				{
									
					if (autoTimer.get() > 1.5) //change num
					{
						gPid.disable();
						myRobot.setLeftRightMotorOutputs(0,0);
						System.out.println("going to 12");
						autoStep = 12; 
					}
					
					break;
				}
				
				case 12: 
				{
					intake.setLeftRightMotorOutputs(-1, -1);					
					
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
					intake.drive(0, 0);
					gPid.disable();
					break; 
				}
			    		
	    	}
    	}
    /*	if (defense.equals("lowbar"))
    	{
    		switch(autoStep)
    		{ 
    			
    			case 0: //start the PID to go straight
    			{
	    		   	tPower = -0.25;
	    			gPid.setSetpoint(0);
	    			gPid.enable();
	    			autoStep = 1; 
	    			System.out.println("going to 1");
	    			adjustArmHeight(LOWBARHEIGHT); 
	    			break; 
    			}
    			
    			case 1:  //goes forward until sees wall on the side
    			{
	    			ultraDistance = inchFromLV(ultrasonic2.getVoltage()); //use sonar to figure out when there
	    			adjustArmHeight(LOWBARHEIGHT);
	    			
	    			if (ultraDistance < 48) //test this number
	    			{
	   					autoStep = 2; 
	   					autoTimer.reset(); 
	   	    			autoTimer.start();
	   	    			System.out.println("going to 2");
	    			}
	    			break;
	    		}
    			
    			case 2: //goes forward for time then goes forward until no more wall on side
	    		{
	    			if (autoTimer.get()>0.2)
	    			{
	    				ultraDistance = inchFromLV(ultrasonic2.getVoltage()); //use sonar to figure out when there
	    				if (ultraDistance > 48)//test this number
	    				{
	    					autoTimer.reset();
	    					autoTimer.start();
	    					autoStep = 3;
	    					System.out.println("going to 3");
	    				}
	    			}
	    			break; 
	    		}
	    		
    			case 3: //goes forward for time so it is over the low bar
	    		{
	    			if (autoTimer.get() > 0.2)
	    			{
	    				autoStep = 4; 
	    				gPid.disable();
	    				myRobot.drive(0, 0); //hold the position - stay
	    				System.out.println("going to 4");
	    			}
	    			break; 
	    		}
    			
    		
    			case 4: // turns to 180 degrees 
	    		{
	    			if (gyro.getAngle()<180)
	    			{
	    				myRobot.setLeftRightMotorOutputs(0.2,-0.2);
	    			
	    			}
	    			else 
	    				autoStep = 5; 
	    			
	    			break; 
	    		}
    		
    			case 5: // turns slowly back 
	    		{
	    			if (gyro.getAngle()>=180)
	    				myRobot.setLeftRightMotorOutputs(-0.2,0.2);
	    			else 
	    				autoStep = 6; 
	    			
	    			break;
	    		}
    		
    			case 6: //uses a PID to start to go forward
	    		{
	    			gPid.setSetpoint(180);
	    			tPower = 0.3; 
	    			gPid.enable();
	    			autoStep=7; 
	    			
	    			break;
	    		}
    		
    		   	    
    			case 7: // waits until the ultrasonic reads the right distance
	    		{
	    			adjustArmHeight(SHOOTHEIGHT); 
	    			if (ultraGoTo (68, ultrasonic, true) == 1)
	    			{
	    				gPid.disable();
	    				autoStep = 8; 
	    			}
	    			
	    			break;
	    		}
    	    
    			case 8: //uses a PID to go backwards
	    	    {
	    	    	tPower = -0.3; 
	    	    	gPid.enable();
	    	    	autoStep = 9;  
	    	    	break;
	    	    }
    	    
    			case 9: // goes backwards until back at the ultrasonic threshold 
	    	    {
	    	    	adjustArmHeight(SHOOTHEIGHT); 
	    	    	if (ultraGoTo (68, ultrasonic, true) == 2)
	    	    	{
	    	    		gPid.disable();
	    	    		autoStep = 10; 
	    	    	}
	    	    	break;
	    	    }
	    	    
    			case 10: // turns to 225 degrees 
	    		{
	    			adjustArmHeight(SHOOTHEIGHT); 
	    			if (gyro.getAngle()<298.36) //test this angle
	    				myRobot.setLeftRightMotorOutputs(0.2,-0.2);
	    			else 
	    				autoStep = 11; 
	    			
	    			break;
	    		}
    		
    			case 11: // turns slowly back 
	    		{
	    			if (gyro.getAngle()>=298.36) //test this angle
	    				myRobot.setLeftRightMotorOutputs(-0.2,0.2);
	    			else 
	    				autoStep = 12; 
	    			
	    			break;
	    		}
    		
    			case 12: //uses a PID to start to go forward
	    		{
	    			myRobot.drive(0, 0);
	    			gPid.setSetpoint(225); //test this angle
	    			tPower = 0.3; 
	    			gPid.enable();
	    			autoStep=13; 
	    			
	    			break;
	    		}
	    		
    			case 13: //stop at the goal, not exactly sure how this will know when to stop 
    			{
    				//if (at the distance)
    				
    				autoTimer.reset();
    				autoTimer.start();
    				autoStep = 14;
    					
    				break;
    			}
    				
    			case 14: //shoot
    			{
    				if (autoTimer.get() < 1)
    				{
    					intake.drive(1, 0);
    				}
    				
    				else 
    					autoStep = 15;
    				
    				break;
    			}
    		}
    	}*/
    			
    	    
    	    
    	/*if (defense.equals("lowbar"))
    	{
    		if (autoStep == 0)
    		{
    			gPid.enable();
    			autoStep = 1; 
    		}
    		
    		if (autoStep == 1)
    		{
    			ultraDistance = inchFromLV(ultrasonic2.getVoltage()); //use sonar to figure out when there
    			adjustArmHeight(LOWBARHEIGHT); 
    			
    			if (ultraDistance < 36) //test this number
    			{
   					autoStep = 2; 
   					autoTimer.reset(); 
   	    			autoTimer.start();
    			}
    			
    		if (autoStep == 2)
    		{
    			if (autoTimer.get()>0.2)
    			{
    				ultraDistance = inchFromLV(ultrasonic2.getVoltage()); //use sonar to figure out when there
    				if (ultraDistance > 36)
    				{
    					autoTimer.reset();
    					autoTimer.start();
    					autoStep = 3; 
    				}
    			}
    		}
    		
    		if (autoStep == 3)
    		{
    			if (autoTimer.get() > 0.15)
    			{
    				autoStep = 4; 
    				gPid.disable();
    				myRobot.drive(0, 0); //hold the position - stay
    				
    			
    		}*/
    		/*
    		if (autoStep == 4)
    		{
    			gPid.setSetpoint(180);
    			gPid.enable();
    			autoStep=5; 
    		}
    		
    		if (autoStep == 5)
    		{
    			if (ultraGoTo (72, ultrasonic, true, -0.3, -0.125)) 
    			{
    				autoStep = 6; 
    				myRobot.drive(0, 0);
    			}
    			
    			 
    		}*/
    		
    	
    		// gyro forward 
    		//use ultrasonic to see when on defense 
    		// use sonar to see when over the defense
    		// drive forward
    		//use sonar to know when to stop
    		//use gyro to turn
    		// drive forward
    		//use sonar and/or run in to castle
    		//shoot
    		//stay
    	
    	
    	
    	if (defense.equals("rockwall") || defense.equals("moat") || defense.equals("ramparts") || defense.equals("rough"))
    	{
    		switch(autoStep)
    		{ 
    			case 0: //start the PID to go straight
    			{
	    		   	tPower = -1;
	    			gPid.setSetpoint(0);
	    			gPid.enable();
	    			autoStep = 1;
	    			break; 
    			}
    			
    			case 1:  //goes forward until sees wall on the side
    			{
	    			ultraDistance = inchFromLV(ultrasonic2.getVoltage()); //use sonar to figure out when there
	    				    			
	    			if (ultraDistance < 36) //test this number
	    			{
	   					autoStep = 2; 
	   					autoTimer.reset(); 
	   	    			autoTimer.start();
	    			}
	    			break;
	    		}
    			
    			case 2: //goes forward for time then goes forward until no more wall on side
	    		{
	    			if (autoTimer.get()>0.2)
	    			{
	    				ultraDistance = inchFromLV(ultrasonic2.getVoltage()); //use sonar to figure out when there
	    				if (ultraDistance > 36)//test this number
	    				{
	    					autoTimer.reset();
	    					autoTimer.start();
	    					autoStep = 3; 
	    				}
	    			}
	    			break; 
	    		}
	    		
    				    		
    			case 3: //goes forward for time so it is over the low bar
	    		{
	    			if (autoTimer.get() > 0.1)
	    			{
	    				autoStep = 4; 
	    				gPid.disable();
	    				myRobot.drive(0, 0); //hold the position - stay
	    			}
	    			break; 
	    		}
	    		
    			case 4: 
    			{
    				startCalibration(); 
    				myRobot.drive(0, 0);
    				intake.drive(0, 0);
    				break;
    			}
    		}
    			
    		
    		
    		// go fast forward 
    		// use sonar to see where it is 
    		// gun it 
    		// use sonar to see when the wall steadily approaches - stop 
    	}
    		
    	if (defense.equals("spy"))
    		
    	{
    		System.out.println("Ultrasonic: " + (inchFromLV(ultrasonic2.getVoltage())));
    		
    		/*switch(autoStep)
    		{ 
    			case 0: //start the PID to go straight
    			{
	    		   	tPower = 0.3;
	    			gPid.setSetpoint(0);
	    			gPid.enable();
	    			autoStep = 1;
	    			break; 
    			}
    			
    			case 1:  //goes forward until sees castle
    			{
	    			ultraDistance = inchFromHRLV(ultrasonic.getVoltage()); //use sonar to figure out when there
	    			adjustArmHeight(SHOOTHEIGHT); 
	    			
	    			if (ultraDistance < 36) //test this number
	    			{
	   					autoStep = 2; 
	   					myRobot.drive(0, 0);
	   					autoTimer.reset();
	   					autoTimer.start();
	   				}
	    			break;
	    		}
    			
    			case 2: //goes forward for time then goes forward until no more wall on side
	    		{
	    			if (autoTimer.get()<1)
	    			{
	    				intake.drive(1, 0);
	    			}
	    			
	    			else
	    				autoStep = 3; 
	    			break; 
	    		}
	    		
    			case 3: //goes forward for time so it is over the low bar
	    		{
	    			myRobot.drive(0, 0);
	    			intake.drive(0, 0);
	    			break;
	    		}*/
    			
    	}
    	
    	if (defense.equals("reach"))
    	{
    		switch(autoStep)
    		{ 
	    		case 0: //start the PID to go straight
				{
				   	tPower = 0.3;
					gPid.setSetpoint(0);
					gPid.enable();
					autoStep = 1; 
					System.out.println("going to 1");
					autoTimer.reset();
   					autoTimer.start();
					break; 
				}
		    	
		    	case 1:
				{
	    			ultraDistance = inchFromHRLV(ultrasonic.getVoltage()); //use sonar to figure out when there
	    				    			
	    			if (autoTimer.get()>0.75) //test this number
	    			{
	   					autoStep = 2; 
	   					System.out.println("going to 2");
	   					
	    			}
	    			break;
	    		}
				
		    	case 2:
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
   
    	if (gPid.isEnabled())
    	 	 gPid.disable(); 
    	
    	myRobot.drive(0, 0);
    	myRobot.setSafetyEnabled(false);
    	autoStep=0; 
    	startCalibration();
    	

    }

    /**
     * This function is called periodically during operator control
     */
    

    
    public enum TeleopFunctions 
    {
    	NONE(0), LOWBAR(1), GRABBALL(2), SHOOT(4), RESET(3), HOLD(4); 
    	
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
    
    public void teleopPeriodic() 
    {
    	System.out.println("UltrasonicHRLV: " + (inchFromHRLV(ultrasonic.getVoltage()))+ "\tUltrasonicLV: " + (inchFromLV(ultrasonic2.getVoltage())) + "\tGyro: " + gyro.getAngle());
    	
    	if (isCalibrating)
    		checkCalibrationStatus(); 
  	   	
    	TeleopFunctions chosen; 
    	int currentPosition = armR.getEncPosition();
    	System.out.println(currentPosition);
    	
    	double scale = stickj.getRawAxis(3)*-1; 
    	
    	scale = ((scale+1)/5)+0.6; 

    	moveValue = stickj.getRawAxis(1);
    	rotateValue = stickj.getRawAxis(0); 
    	
    	if (Math.abs(moveValue)<0.005)
    		moveValue = 0; 
    	
    	if (Math.abs(rotateValue)<0.005 && Math.abs(moveValue)<0.1)
    		rotateValue = 0;
    	
    	moveValue = moveValue*scale; 
    	rotateValue = rotateValue*scale; 
    	
    	
    	myRobot.arcadeDrive(moveValue, rotateValue, true); //this causes the robot to be controlled by the other joystick
    	
    	double intakeAxis = stickx.getRawAxis(5); 
    	if(Math.abs(intakeAxis)<0.25) 
    		intakeAxis=0; 
    		
    	intake.drive(intakeAxis, 0);
    	
    	if(stickj.getRawButton(1))
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
    	}
    	
		activeCamera.getImage(img);
		
		server.setImage(img); // puts image on the dashboard
			
       	if (stickj.getRawButton(2))
       	{  	     		
       		
        	inverted=!inverted; 
        	
        	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kFrontLeft, inverted);
        	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kRearLeft, inverted);
        	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kFrontRight, inverted);
        	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kRearRight, inverted);
    
       	}
       	
        //The buttons on the xBox are Y(top, 3) B(right,2) A(bottom, 1) X(left, 4)     
      
      	chosen = TeleopFunctions.NONE;     	
    	
        //Y is for the calibration 
        if (stickx.getRawButton(3)==true)
        	chosen = TeleopFunctions.SHOOT; 
        

        //A is for getting the ball to the right place for crossing low bar
        else if (stickx.getRawButton(1)==true)
        	chosen = TeleopFunctions.LOWBAR; 
        
  
       //X is for shooting 
        else if (stickx.getRawButton(4)==true)
            chosen = TeleopFunctions.RESET;
        
        //B is for getting the ball to the right place for intake 
        else if (stickx.getRawButton(2)==true)
        	chosen = TeleopFunctions.GRABBALL;
        
        //Upper Right Button is for holding the intake arm in its place
        else if (stickx.getRawButton(6)==true)
        	chosen = TeleopFunctions.HOLD;
        
           
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
	        	
	        case LOWBAR: 
	        	adjustArmHeight(LOWBARHEIGHT); 
	        	break; 
	        	
	        case SHOOT:
	        	adjustArmHeight(SHOOTHEIGHT); 
	        	break; 
	        	
	        case HOLD: 
	        	adjustArmHeight(armR.getPosition()); 
	        	break; 
        }
        
        if(!isCalibrating&&chosen!=TeleopFunctions.HOLD) {
    		double armAxis = stickx.getRawAxis(1); 

    		if(armR.getControlMode()==TalonControlMode.Position)
    		{
    			System.out.println("EXITING POSITION MODE");
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
    	isCalibrating=true; 
    	isCalibrated = false; 
    	armR.changeControlMode(TalonControlMode.PercentVbus);
    	lastPosition = armR.getPosition();
    	calTimer.start();
    	armR.set(0.6);
    	
    }
    
    public boolean checkCalibrationStatus()
    {
    	double timerVal = calTimer.get();
/*    	System.out.println("calTimer:" + timerVal);*/
    	if (timerVal<0.5)
    		return false; 
    	
		double newPosition = armR.getPosition(); 
		/*System.out.println("cDelta:" + (newPosition-lastPosition));*/
	
		
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
    
    public void adjustArmHeight (double height)
    {
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
    
    public double inchFromHRLV (double volts)
    {
    	double v = (volts*43.796)-2.6048; 
    	return v; 
    }
    
    public double inchFromLV (double volts)
    {
    	 double v = (volts*91.019) + 5.0692; 
    	 return v; 
    }
    

    public boolean turn (double angle, boolean right)
    {
    
    	
    	if (gyro.getAngle()<angle+2)
    	{
    		myRobot.drive(0.2, 1);
    		return false; 
    	}
    	
    	else if (gyro.getAngle()>angle-2)
    	{
    		myRobot.drive(0.2, 1);
    		return false; 
    	}
    	
    	else 
    		return true; 
    }
    
    //returns 0 if it has not gotten to the right distnace
    //returns 1 if it has seen it once and needs to go backwards
    //returns 2 if it is done doing its thing
    public int ultraGoTo (double distance, AnalogInput ultra, boolean HRLV)
    { 
    	 	    	    	
    	if (HRLV)
    		ultraVal = inchFromHRLV(ultra.getVoltage()); 
    	else 
    		ultraVal = inchFromLV(ultra.getVoltage()); 
    	
    	/*System.out.println("Current Ultra View: " + ultraVal);*/
    	
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
  //internal class to write to myRobot (a RobotDrive object) using a PIDOutput
    public class GyroPIDOutput implements PIDOutput 
    {
    	int counter = 0; 
    	   	
    	public void setPower(double power)
    	{
    		tPower = power; 
    	}
    	
	    public void pidWrite(double output) 
	    {
	    		   
	
	    	double scaled = output*0.1;
	    	
	    	myRobot.setLeftRightMotorOutputs(-1*(tPower+scaled), -1*(tPower-scaled));
	    	/*System.out.println("Left wheel: " + (-(tPower+scaled)));
	    	System.out.println("Right wheel: " + (-(tPower-scaled)));*/
	    	timesLoop++; 
	    	
	    		    
	    }
    }
    
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

    
    
}
