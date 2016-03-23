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
    int cameraID = 1; 
	public static USBCamera cameraFront;
/*	public static USBCamera cameraBack;*/
	public static USBCamera activeCamera; 
	
	int turnCount;
	
	boolean inverted; 
	
	int autoStep; 

	boolean isCalibrating = false; 
    double lastPosition; 
    Timer calTimer = new Timer(); 
    
    boolean isTurning = false; 
	boolean	isTurned = false;
	boolean turnBack = false; 
	double holdPosition = 180; 
   
    
	double tPower; 
	int teleopFunction; 
	
	CANTalon armR; //this is the arm that rotates the ball intake
	
	Timer teleTimer = new Timer();
	
	PIDController gPid; 
	
	private final int INTAKEHEIGHT = -69000; 
	private final int LOWBARHEIGHT = -62000; 
	private final int LOWBARFORWARDHEIGHT = -60000; 
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
		cameraFront = new USBCamera("cam1"); //changed from cam0 3-17
/*		cameraBack = new USBCamera("cam1");*/
		cameraFront.openCamera();
/*		cameraBack.openCamera();*/
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

    	
  		autoStep = 0; 
  		
  		defense = chooser.getSelected().toString();
 		
   
  		if (defense.equals("lowbar")) {
  			System.out.println("RUNNING LOW BAR");
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
		if (counter++%1 ==0 )
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
					adjustArmHeight(-58000); 
					break; 
				}
		    	
		    	case 1:
    			{
	    			ultraDistance = inchFromHRLV(ultrasonic.getVoltage()); //use sonar to figure out when there
	    			adjustArmHeight(-58000);
	    			
	    			if (ultraDistance < 24) //test this number
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
		    		ultraDistance = inchFromHRLV(ultrasonic.getAverageVoltage()); 
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
	    				tPower = 0.175;
	    				gPid.enable();
	    				System.out.println(ultraDistance); 
	    				System.out.println("going to 5");
	    			}
	    			
	    			break;
	    		}
	    		
				case 30: 
				{
					ultraDistance = inchFromHRLV(ultrasonic.getVoltage());
					
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
	    	    	tPower = -0.175; 
	    	    	gPid.enable();
	    	    	System.out.println("going to 6");
	    	    	autoStep = 6;  
	    	    	break;
	    	    }
		    
				case 6: // goes backwards until back at the ultrasonic threshold 
	    	    {
	    	    	ultraDistance = inchFromHRLV(ultrasonic.getVoltage()); 
	    	    	
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
	    			adjustArmHeight(SHOOTHEIGHT); 
	    			if (gyro.getAngle()< 58) //test this angle
	    				myRobot.setLeftRightMotorOutputs(-0.3,0.3);
	    			else 
	    			{
	    				autoStep = 8; 
	    				System.out.println("going to 8");
	    				myRobot.setLeftRightMotorOutputs(0,0);
	    			}
	    			
	    			break;
	    		}
			
				case 8: // turns slowly back 
	    		{
	    			if (gyro.getAngle() >= 58) //test this angle
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
									
					if (autoTimer.get() > 2) //change num
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
				case 14: 
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
   
    	if (gPid != null && gPid.isEnabled())
    	 	 gPid.disable(); 
    	
    	myRobot.drive(0, 0);
    	myRobot.setSafetyEnabled(false);
    	autoStep=0; 
    	

    }

    /**
     * This function is called periodically during operator control
     */
    

    
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
    
    public void teleopPeriodic() 
    {
    	System.out.println("UltrasonicHRLV: " + (inchFromHRLV(ultrasonic.getVoltage())) + "  Gyro: " + gyro.getAngle());
    	
    	
    	if (isCalibrating)
    		checkCalibrationStatus(); 
    	
    	if (isTurning)
    	{
    		checkTurningStatus(); 
    		return; 
    	}
  	   	
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
    	
    	if (rotateValue > 0)
    		isTurning = false; 
    	
    	
    	if (!isTurning) 
    		myRobot.arcadeDrive(moveValue, rotateValue, true); //this causes the robot to be controlled by the other joystick
    	
    	double intakeAxis = stickx.getRawAxis(5); 
    	if(Math.abs(intakeAxis)<0.25) 
    		intakeAxis=0; 
    		
    	intake.drive(intakeAxis, 0);
    	
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
    	
    	if (!isTurning)
    	{
	    	if(stickj.getRawButton(2))
	    		startTurning();
    	}
    	
    	if (stickj.getRawButton(5)) // see if Jack likes this button 
    		isTurning = false; 
    	
		activeCamera.getImage(img);
		
		server.setImage(img); // puts image on the dashboard
			
       	if (stickj.getRawButton(3)) //changed 3.14 from button 2 to button 3
       	{  	     		
       		
        	inverted=!inverted; 
        	
        	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kFrontLeft, inverted);
        	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kRearLeft, inverted);
        	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kFrontRight, inverted);
        	myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kRearRight, inverted);
    
       	}
       	
        //The buttons on the xBox are Y(top, 3) B(right,2) A(bottom, 1) X(left, 4)     
      //angela was here
      	chosen = TeleopFunctions.NONE;     	
    	
        //Y is for the calibration 
        if (stickx.getRawButton(3)==true)
        	chosen = TeleopFunctions.SHOOT; 
        

        //A is for getting the ball to the right place for crossing low bar
        else if (stickx.getRawButton(1)==true)
        	chosen = TeleopFunctions.LOWBARBACKWARD; 
        
  
       //X is for shooting 
        else if (stickx.getRawButton(5)==true)
            chosen = TeleopFunctions.RESET;
   
        
        //B is for getting the ball to the right place for intake 
        else if (stickx.getRawButton(2)==true)
        	chosen = TeleopFunctions.GRABBALL;
        
        //Upper Right Button is for holding the intake arm in its place
        else if (stickx.getRawButton(6)==true)
        	chosen = TeleopFunctions.HOLD;
        
        else if (stickx.getRawButton(4)== true)
        	chosen = TeleopFunctions.LOWBARFORWARDS; 
        
           
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
    	armR.set(0.35);
    	
    }
    
    public boolean checkCalibrationStatus()
    {
    	double timerVal = calTimer.get();
/*    	System.out.println("calTimer:" + timerVal);*/
    	if (timerVal<0.1)
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
    
    public void startTurning()
    {
    	gyro.reset();
    	isTurning=true; 
    	isTurned = false; 
    	turnBack = false; 
    	holdPosition = gyro.getAngle()+180; //theoretically should be 180
    	myRobot.setLeftRightMotorOutputs(-0.5, 0.5);
    	
    }
    
 
    
    public boolean checkTurningStatus()
    {
    	    	
		double newPosition = gyro.getAngle(); 
		
		if (turnBack)
		{
			myRobot.setLeftRightMotorOutputs(0.25, -0.25);
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
				myRobot.setLeftRightMotorOutputs(-0.35, 0.35);
			
			if (newPosition > holdPosition) 
			{
				turnBack = true; 
				
			}
		}
		
		
		
		
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
	    	
	    	if(gPid.isEnabled())//this if was added 3-3
	    		myRobot.setLeftRightMotorOutputs(-1*(tPower+scaled), -1*(tPower-scaled));
	    	
	    	/*System.out.println("Left wheel: " + (-(tPower+scaled)));
	    	System.out.println("Right wheel: " + (-(tPower-scaled)));*/
	    	
	    		    
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

    // wo fei chang bu xiang ta men, wo jue de ta men zuo dong xi zuo di bu hao. ta men bu ke yi ting de dong wo men. 
    //ta men de lian shi zuo ta men de ji qi ren dan shi wo men de dui ren zuo wo men de ji qi ren suo yi ta men de 
    //ji qi ren bi wo men de ji qi ren hao yi dian dian dan shi wo men de dui bi ta men de dui fei chang hao
    //ta men shi di ba ge, wo men shi di shi jiu ge. wo jue de bu zhen de ying wei ta men bu zuo na ge. wo jue de ta 
    //men fei chang yue ben. fei chang, fei chang, fei chang bu hao! 
    
    //ni men de auto code bu ke yi duo hen duo dong xi. wo men fei chang bu xi huan ta men. wo bu xi huan ta men. 
    
    //zhi ge fei chang tiao qi. wo men de jia zai ta men de jia de you bian. suo yi mei ge shi hou, wo dei gen ta shuo huo. 
    
}
