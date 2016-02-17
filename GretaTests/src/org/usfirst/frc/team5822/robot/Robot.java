package org.usfirst.frc.team5822.robot;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
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
	Encoder encoderArm; 
	AnalogInput ultrasonic; 
    public static final int NONE = 0; 
    public static final int LOWBAR = 1; 
    public static final int GRABBALL = 2; 
    public static final int CALIBRATE = 3;
    public static final int SHOOT = 4; 
    public int timesLoop = 0; 
    CANTalon testMotor = new CANTalon(0); 

	double tPower; 
	int teleopFunction; 
	
	VictorSP armR; //this is the arm that rotates the ball intake
	
	Timer teleTimer = new Timer();
	
	PIDController gPid; 
	
	int intakeHeight; 
	int lowBarHeight; 
	int shootHeight; 

	
	 CameraServer server;

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
    	
   	
    	gyro = new ADXRS450_Gyro();
    	gyro.calibrate();
    	
    	armR = new VictorSP (9); 
    	encoderArm = new Encoder(0,1);
    	ultrasonic = new AnalogInput(0);

    	
    	//sets up the automatic heights for button functions 
    	intakeHeight = 0; 
    	shootHeight = 0; 
    	lowBarHeight = 0; 
    	

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
    }
  	   	   	
    


    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() 
    {
        		
    	    	    	   	
    }		
    	
    
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit()
    {
   
    	teleopFunction = 0;
    	/*gPid.disable();*/
    	
    }

    /**
     * This function is called periodically during operator control
     */
    

    
    public enum TeleopFunctions 
    {
    	NONE(0), LOWBAR(1), GRABBALL(2), SHOOT(4), CALIBRATE(3); 
    	
    	private int val; 
    	
    	private TeleopFunctions(int val){
    		this.val = val; 
    	}
    }
    
    
    
    TeleopFunctions chosen; 
    double voltage; 
    double value; 
   
  
    public void teleopPeriodic() 
    {
    	/*voltage = ultrasonic.getVoltage();
    	System.out.println((voltage*43.796)-2.6048);
    	    	
    	Timer.delay(1);*/
	
    	myRobot.setSafetyEnabled(false);
  	
        
    	intake.drive(stickx.getRawAxis(5), 0); //this causes the intake to be controlled by the analog stick on the right
    	armR.set(stickx.getRawAxis(1)); //this causes the rotating arm to be controlled by the analog stick on the left 
     
    	myRobot.arcadeDrive(stickj); //this causes the robot to be controlled by the other joystick 
    	
    	testMotor.set(1);
    	
/*    	
        //The buttons on the xBox are Y(top, 3) B(right,2) A(bottom, 1) X(left, 4)     
      
       
        
        //Y is for the calibration 
        if (stickx.getRawButton(3)==true)
        	chosen = TeleopFunctions.CALIBRATE; 
        

        //A is for getting the ball to the right place for crossing low bar
        else if (stickx.getRawButton(1)==true)
        	chosen = TeleopFunctions.LOWBAR; 
        
  
       //X is for shooting 
        else if (stickx.getRawButton(4)==true)
        	chosen = TeleopFunctions.SHOOT; 
        
        //B is for getting the ball to the right place for intake 
        else if (stickx.getRawButton(2)==true)
        	chosen = TeleopFunctions.GRABBALL;
        
        else 
        	chosen = TeleopFunctions.NONE; 
      
        
        switch (chosen)
        {
    	//nothing happens, the arm has not been signaled 
        //this is at the beginning so the case statement will break if there is no instruction
        case NONE: teleopFunction = 0; 
        		break; 
        	
        		
        case CALIBRATE: //this is for calibration
        {
        	if (encoderArm.get()>0)
        	{
        		armR.set(1); //rotates the arm up
        	}
        	
        	if(encoderArm.get()<=-0.1 || encoderArm.get()>= 0.1) //this sets a bound on either side so robot doesn't have to be perfect
        	{
        		armR.set(1);
        		teleTimer.start();
        	}
        	
        	//jack was here, he did not contribute
        	
        	if (teleTimer.get()>0 && teleTimer.get()<0.5)//adjust time as needed
        		armR.set(1);
        	
        	if (teleTimer.get()>0.5)
        	{
        		if(encoderArm.get()<=-0.1 || encoderArm.get()>= 0.1) //this sets a bound on either side, for error - adjust
				{
	        		armR.set(0);
	        		teleTimer.reset(); 
	        		chosen = TeleopFunctions.NONE; //arm finishes calibrating so teleopFunction reset to default
				}
        		
        		else 
        		{
        			teleTimer.reset();
        		}
        	}
        		
        }
        
        case LOWBAR: //this gets the arm to the right place for the low bar
        {
        	if (encoderArm.get()<lowBarHeight-0.1) //make sure 0.1 is good number, adjust once the encoder is on
        		armR.set(1);
        	
        	else if (encoderArm.get()>lowBarHeight+0.1) //0.1 makes sure the robot doesn't have to be completely accurate
        		armR.set(-1); 
        	
        	else 
        	{
        		armR.set(0);
        		chosen = TeleopFunctions.NONE; //arm is at iH so teleopFunction back to default
        	}
        }
        
        case SHOOT: //this gets the arm to the right place for the low bar
        {
        	if (encoderArm.get()<shootHeight-0.1) //make sure 0.1 is good number, adjust once the encoder is on
        		armR.set(1);
        	
        	else if (encoderArm.get()>shootHeight+0.1) //0.1 makes sure the robot doesn't have to be completely accurate
        		armR.set(-1); 
        	
        	else 
        	{
        		armR.set(0);
        		chosen = TeleopFunctions.NONE; //arm is at iH so teleopFunction back to default
        	}
        }
        case GRABBALL: teleopFunction = 3; //this gets the arm to the right place for intake 
        {
        	if (encoderArm.get()<intakeHeight-0.1) //make sure 0.1 is good number, adjust once the encoder is on
        		armR.set(1);
        	
        	else if (encoderArm.get()>intakeHeight+0.1) //0.1 makes sure the robot doesn't have to be completely accurate
        		armR.set(-1); 
        	
        	else 
        	{
        		armR.set(0);
        		chosen = TeleopFunctions.NONE; //arm is at iH so teleopFunction back to default
        	}
        }
      }   */
    }
       
                     
                
   		
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    } //End Test Periodic 
    
}
