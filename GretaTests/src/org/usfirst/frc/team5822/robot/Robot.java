package org.usfirst.frc.team5822.robot;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

	double tPower; 
	int teleopFunction; 
	VictorSP armR = new VictorSP (7); //this is the arm that rotates the ball intake
	Timer teleTimer = new Timer();

	//adding an encoder
	Encoder eArm;
	/*VictorSP arm = new VictorSP (6);  
	VictorSP rotator = new VictorSP (5);*/  
	

	PIDController gPid; 
	

	int intakeHeight; 
	int passHeight; 
	int shootHeight; 

	
	 CameraServer server;

	//internal class to write to myRobot (a RobotDrive object) using a PIDOutput
	    public class GyroPIDOutput implements PIDOutput 
	    {
		    public void pidWrite(double output) 
		    {
		    /*	myRobot.drive(output, 0); //drive robot from PID output
		   
*/		    	
		    	if (output < -0.2)
		    		output = -0.2; 
		    		
		    	if (output > 0.2)
		    		output = 0.2;
		    	
		    	myRobot.setLeftRightMotorOutputs(tPower-output, tPower+output);
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
    	/*motorButtonA = new JoystickButton(stick, 1);
    	motorButtonB = new JoystickButton(stick, 2);*/
    	
    	server = CameraServer.getInstance();
        server.setQuality(50);
        //the camera name (ex "cam0") can be found through the roborio web interface
        server.startAutomaticCapture("cam0");  
        
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
    	
 	    //encoder code 2.1 Greta Rauch
    	eArm = new Encoder (0,1,false, Encoder.EncodingType.k4X); 
    	eArm.setDistancePerPulse(4);
    	
    	gyro = new ADXRS450_Gyro();
    	gyro.calibrate();

    	
    	//sets up the automatic heights for button functions 
    	intakeHeight = 0; 
    	shootHeight = 0; 
    	passHeight = 0; 

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
  	    double pGain = 0.1;
  	    double iGain = 0; 
  	    double dGain = 0; 
  	    tPower = 0.2; 
  		gPid = new PIDController(pGain, iGain, dGain, new GyroPIDSource(), new GyroPIDOutput());
  		
    }
  	   	   	
    


    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() 
    {
    	gPid.setInputRange(-360, 360);  
    	gPid.setSetpoint(0);
    	System.out.println(gyro.getAngle());
    	if (isAutonomous())
    		gPid.enable();
    	else 
    		gPid.disable();
    	
    }		
    	
    
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit()
    {
   
    	teleopFunction = 0; 
     	
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {

    	
       intake.arcadeDrive(stickx);
       myRobot.arcadeDrive(stickj);
        
        myRobot.setSafetyEnabled(false);
        
        //button 1 is for the calibration 
        if (stickx.getRawButton(1)==true)
        	teleopFunction = 1; 
         
        //button 2 is for getting the ball to the right place for intake 
        if (stickx.getRawButton(2)==true)
        	teleopFunction = 3; 
        
        //button 3 is for getting the ball to the right place for crossing low bar
        if (stickx.getRawButton(3)==true)
        	teleopFunction = 4; 
        
        //button 4 is for passing the ball 
        if (stickx.getRawButton(4)==true)
        	teleopFunction = 5; 
        
        //button 5 is for shooting 
        if (stickx.getRawButton(5)==true)
        	teleopFunction = 6; 
        
        switch (teleopFunction)
        {
        case 1: teleopFunction = 0; 
        		break; 
        		//nothing happens, the arm has not been signaled 
        case 2: teleopFunction = 1; //this is for calibration
        {
        	if (eArm.get()>0)
        	{
        		armR.set(1); //rotates the arm up
        	}
        	
        	if(eArm.get()<0)
        	{
        		armR.set(1);
        		teleTimer.start();
        		teleopFunction = 2; //starts a timer
        	}
        		
        }
        case 3: teleopFunction = 2; //has the arm finish calibrating for time 
        {
        	if (teleTimer.get()<0.5)
        		armR.set(1);
        	if (teleTimer.get()>0.5)
        	{
        		armR.set(0);
        		teleTimer.reset(); 
        		teleopFunction = 0; //arm finishes calibrating so teleopFunction reset to default
        	}
        }
        
        case 4: teleopFunction = 3; //this gets the arm to the right place for intake 
        {
        	if (eArm.get()<intakeHeight-0.1) //make sure 0.1 is good number, set iH
        		armR.set(1);
        	
        	else if (eArm.get()>intakeHeight+0.1) //make sure 0.1 is good number
        		armR.set(-1); 
        	
        	else 
        	{
        		armR.set(0);
        		teleopFunction = 0; //arm is at iH so teleopFunction back to default
        	}
        }
        
        case 5: 
        
        }
        if (stickx.getRawButton(1)==true)
        {
        	
        }
                     
                
   		/*while(isEnabled()){
   			//buttonPressedA = motorButtonA.get();
   	       // buttonPressedB = motorButtonB.get();
        	double currentSpeed = 0;


    	
     			
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
*/   		}//End While isEnabled
 
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    } //End Test Periodic 
    
}
