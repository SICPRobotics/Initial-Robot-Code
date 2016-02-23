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
    	
    	ultrasonic = new AnalogInput(0);
    	ultrasonic2 = new AnalogInput(1); 
    	
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
  	
    	   

    }//End robotInit
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit()  
    {    	
    
  	   	System.out.println("We have been through autonomousInit");
  	    gyro.reset();
  	    System.out.println("We have reset gyro"); 
  	    double pGain = 0.9;
  	    double iGain = 0.00521135; 
  	    double dGain = 38.834084496; 
  	    PIDSource gType = new GyroPIDSource ();
  	    GyroPIDOutput gOutput = new GyroPIDOutput(); 
   	    gType.setPIDSourceType(PIDSourceType.kDisplacement);
  		gPid = new PIDController(pGain, iGain, dGain, gType, new GyroPIDOutput(), 0.001); //the lowest possible period is 0.001
    	gPid.setInputRange(-360, 360);  
    	gPid.setSetpoint(0);
/*  		gPid.enable();*/
  		teleTimer.reset();
  		teleTimer.start();
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
  		
  		turnCount = 0; 
  		startCalibration(); 	
    }
  	   	   	
    
  

    /**
     * This function is called periodically during autonomous
     */
    
    boolean next = false; 
    
    public void autonomousPeriodic() 
    {
    	double ultraDistance; 
    	
    	if (isCalibrating)
    		checkCalibrationStatus(); 
    	
    	/*if (autoStep ==0)
    	    if (ultraGoTo (24, ultrasonic, true, -0.175, -0.12))
    	    	autoStep = 1; 
    	*/
    	if (defense.equals("lowbar"))
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
    				
    			
    		}
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
    		
    	}
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
    		// go fast forward 
    		// use sonar to see where it is 
    		// gun it 
    		// use sonar to see when the wall steadily approaches - stop 
    	}
    		
    	if (defense.equals("spy"))
    		
    	{
    		//drive straight forward
    		// use sonar to see castle
    		//shoot 
    		//stay 
    	}
    	
    	/*if (defense.equals("reach"))
    	{
    		if (autoStep == 0)
    		{
    			gPid.enable();
    			autoStep = 1; 
    		}
    		
    		if (autoStep == 1)
    		{
    			myRobot.drive(0.2, 0); //drive forward slowly
    			
    			ultraDistance = inchFromLV(ultrasonic2.getVoltage()); //use sonar to figure out when there
    			
    			if (ultraDistance < 24) //test this number
    			{
    				myRobot.drive(0,0);
    				autoStep = 2; 
    			}
    			
    		if (autoStep == 2)
    		{
    			autoTimer.reset(); 
    			autoTimer.start();
    			autoStep = 3; 
    		}
    		
    		if (autoStep == 3)
    		{
    			if (autoTimer.get() < 0.5)
    				myRobot.drive(0.175, 0);
    			
    			else 
    			{
    				autoStep = 4; 
    				myRobot.drive(0, 0); //hold the position - stay
    			}
    		}

    	}*/
    	}
    	} 
    }
    	
    
    /*
     * This function is called once each time the robot enters tele-operated mode*/
        
   
    public void teleopInit()
    {
   
    	/*if (gPid != null) gPid.disable();*/
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
    	
    	if (isCalibrating)
    		checkCalibrationStatus(); 
  	
    	System.out.println("Encoder Position: " + armR.getPosition()); 
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
    } //End Test Periodic 
    
    public void startCalibration()
    {
    	isCalibrating=true; 
    	isCalibrated = false; 
    	armR.changeControlMode(TalonControlMode.PercentVbus);
    	lastPosition = armR.getPosition();
    	calTimer.start();
    	armR.set(0.3);
    	
    }
    
    public boolean checkCalibrationStatus()
    {
    	double timerVal = calTimer.get();
    	System.out.println("calTimer:" + timerVal);
    	if (timerVal<0.5)
    		return false; 
    	
		double newPosition = armR.getPosition(); 
		System.out.println("cDelta:" + (newPosition-lastPosition));
	
		
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
    
 
    
    public boolean ultraGoTo (double distance, AnalogInput ultra, boolean HRLV, double speedTo, double speedBack)
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
    
  //internal class to write to myRobot (a RobotDrive object) using a PIDOutput
    public class GyroPIDOutput implements PIDOutput 
    {
    	int counter = 0; 
    	double tPower= -0.3;
    	
    	public void setPower(double power)
    	{
    		tPower = power; 
    	}
    	
	    public void pidWrite(double output) 
	    {
	    		   
	
	    	double scaled = output*0.1;
	    	
	    	myRobot.setLeftRightMotorOutputs(-1*(tPower+scaled), -1*(tPower-scaled));
	    	System.out.println("Left wheel: " + (-(tPower+scaled)));
	    	System.out.println("Right wheel: " + (-(tPower-scaled)));
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
    		  System.out.println(gyro.getAngle());
    		  return gyro.getAngle();
    		  
    		  
    	  }
    	}

    
    
}
