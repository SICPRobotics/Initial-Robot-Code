package org.usfirst.frc.team5822.robot;


import java.lang.reflect.Array;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
/*import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;*/
import edu.wpi.first.wpilibj.vision.USBCamera;
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
	SICPRobotDrive intake;
	Joystick stickx;
	Joystick stickj; 
	int autoSequenceCounter;
	int gyroCounter; 
	double speedCountTest; 
	double Kp = 0.03; 
	boolean buttonPressedA;
    boolean buttonPressedB;
	JoystickButton motorButtonA;		
	JoystickButton motorButtonB;	
	Servo servo1;
	Timer autoTimer = new Timer();
	ADXRS450_Gyro gyro;
	AnalogInput ultrasonic; 
    public int timesLoop = 0; 
    int cameraID = 0; 
	public static USBCamera cameraFront;
	public static USBCamera cameraBack;
	public static USBCamera activeCamera; 
	
	boolean isCalibrating = false; 
    double lastPosition; 
    Timer calTimer = new Timer(); 
   
    
	double tPower; 
	int teleopFunction; 
	
	CANTalon armR; //this is the arm that rotates the ball intake
	
	Timer teleTimer = new Timer();
	
	PIDController gPid; 
	
	private final int INTAKEHEIGHT = -70000; 
	private final int LOWBARHEIGHT = -63500; 
	private final int SHOOTHEIGHT = -54000; 
	Image img =  NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0); 

	
	 CameraServer server;
	 
/*	 SendableChooser chooser; */

	//internal class to write to myRobot (a RobotDrive object) using a PIDOutput
	    public class GyroPIDOutput implements PIDOutput 
	    {
	    	int counter = 0; 
	    	
		    public void pidWrite(double output) 
		    {
		    		   
   	
		    	double scaled = output*0.15;
		    	
		    	myRobot.setLeftRightMotorOutputs(-1.0*(tPower+scaled), -1*(tPower-scaled));
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
	    		  return gyro.getAngle();
	    		  
	    	  }
	    	}

	    
	    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	 public void robotInit() {
    	
		server = CameraServer.getInstance();
        server.setQuality(50);
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
    	
    	//sets up intake
    	
    	intake = new SICPRobotDrive(5, 6);
	
    	//sets up joysticks
    	stickj = new Joystick(0);  
    	stickx = new Joystick(1); 
    	
   	
    	gyro = new ADXRS450_Gyro();
    	gyro.calibrate();
    	
    	ultrasonic = new AnalogInput(0);
    	
    	armR = new CANTalon(1); 
    	armR.changeControlMode(TalonControlMode.Position); //Change control mode of talon, default is PercentVbus (-1.0 to 1.0)
    	armR.setFeedbackDevice(FeedbackDevice.QuadEncoder); //Set the feedback device that is hooked up to the talon
    
    	armR.setPID(0.2, 0.001, 100, 0.00, 360, 12, 0); //Set the PID constants (p, i, d)
    	armR.reverseSensor(true);

    	armR.changeControlMode(TalonControlMode.PercentVbus); //Change control mode of talon, default is PercentVbus (-1.0 to 1.0)

/*    	armR.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	armR.changeControlMode(TalonControlMode.Position);*/
    	
    	/*armR.changeControlMode(TalonControlMode.Position); //fChange control mode of talon, default is PercentVbus (-1.0 to 1.0)
    	armR.setFeedbackDevice(FeedbackDevice.QuadEncoder); //Set the feedback device that is hooked up to the talon
    	armR.setPID(0.5, 0.001, 0.0); //Set the PID constants (p, i, d)
    	armR.enableControl(); //Enable PID control on the talon
*/    	
    	/*SendableChooser chooser = new SendableChooser();
    	chooser.initTable(NetworkTable.getTable("Defense Chooser"));
    	chooser.addDefault("Low Bar", "lowbar");
    	chooser.addObject("Ramparts", "ramparts");
    	chooser.addObject("Moat", "moat");
    	chooser.addObject("Cheval de Frise", "cheval");
    	chooser.addObject("Rock Wall", "rockwall");
    	//ect...add the rest of the defenses

    	SmartDashboard.putData("Autonomous Defense Chooser", chooser);
  	*/

    }//End robotInit
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit()  
    {    	
    	autoSequenceCounter = 0;
  	   	System.out.println("We have been through autonomousInit");
  	    gyro.reset();
  	    System.out.println("We have reset gyro"); 
  	    double pGain = 0.9;
  	    double iGain = 0.00521135; 
  	    double dGain = 38.834084496; 
  	    tPower = 0.2;
  	    PIDSource gType = new GyroPIDSource ();
  	    gType.setPIDSourceType(PIDSourceType.kDisplacement);
  		gPid = new PIDController(pGain, iGain, dGain, gType, new GyroPIDOutput(), 0.001); //the lowest possible period is 0.001
    	gPid.setInputRange(-360, 360);  
    	gPid.setSetpoint(0);
  		gPid.enable();
  		teleTimer.reset();
  		teleTimer.start();
  		timesLoop=0; 
  		
/*  		String defense = chooser.getSelected().toString();*/
  		
    }
        
  		/*if (defense.equals("lowbar")) {
  			System.out.println("RUNNING LOW BAR");
  		} else if (defense.equals("ramparts")) {
  			System.out.println("RUNNING RAMPARTS");
  		} else if (defense.equals("moat")) {
  			System.out.println("RUNNING MOAT");
  		} else if (defense.equals("cheval")) {
  			System.out.println("RUNNING CHEVAL");
  		} else if (defense.equals("rockwall")){
  			System.out.println("RUNNING ROCKWALL");
    }
  			
    }*/
  	   	   	
    


    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() 
    {
        		
    	    	    	   	
    }		
    	
    
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    
    int count=0; 
    public void teleopInit()
    {
   
    	/*gPid.disable();*/
    	myRobot.setSafetyEnabled(false);
    	count=0; 
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
    
    public void teleopPeriodic() 
    {
    	/*voltage = ultrasonic.getVoltage();
    	System.out.println((voltage*43.796)-2.6048);
    	    	
    	Timer.delay(1);*/
    	
    	if (isCalibrating)
    		checkCalibrationStatus(); 
  	
    	TeleopFunctions chosen; 
    	int currentPosition = armR.getEncPosition();
    	System.out.println(currentPosition);
    	
 /*   	if (firstTime) {
    	//armR.enableControl(); //Enable PID control on the talon
    	System.out.println("FIRSTIME");
//    	armR.setPosition( 9158); 
    	armR.setPosition( 258);
    	//armR.set(60000);
    	firstTime = false;
    	}*/

    	myRobot.arcadeDrive(stickj); //this causes the robot to be controlled by the other joystick
    	
    	double intakeAxis = stickx.getRawAxis(5); 
    	if(Math.abs(intakeAxis)<0.25) 
    		intakeAxis=0; 
    		
    	intake.drive(intakeAxis, 0);
		
    	//armR.set(stickx.getRawAxis(1));
    	
    	if(stickj.getRawButton(2))
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
    		System.out.println("Enabling Camera: " + camR); 
    		 
    		
    		while(stickx.getRawButton(5));
    	}
    	
		activeCamera.getImage(img);
		
		server.setImage(img); // puts image on the dashboard
		
		
   	
   
    	
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
        
/*        switch (chosen)
        {
    	//nothing happens, the arm has not been signaled 
        //this is at the beginning so the case statement will break if there is no instruction
        case NONE: 
        	
        	//these are in here because they should only happen if other instructions are not given 
        	intake.drive(stickx.getRawAxis(5), 0); //this causes the intake to be controlled by the analog stick on the right
        	//armR.set(stickx.getRawAxis(1)); //this causes the rotating arm to be controlled by the analog stick on the left 
        	armR.set(5000); //Tells the talon to go to 5000 encoder counts, using the preset PID parameters.
        	break; 
        	
        		
           
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

}
