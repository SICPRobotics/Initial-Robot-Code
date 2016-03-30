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

	Joystick xboxCtr; //xbox
	Joystick joystick; //joystick

	int gyroCounter; 
	double speedCountTest; 

	//auto variables
	Timer autoTimer = new Timer();
	int autoStep;
	double autoTurnTo; 
	double autoDistanceTo; 

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

	int turnCount;
	boolean inverted; 

	boolean isCalibrated = false; 
	double moveValue=0; 
	double rotateValue = 0; 

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
	TeleopFunctions lastChosen = TeleopFunctions.NONE;

	//encoder values for heights for the ball intake	
	private final int INTAKEHEIGHT = -69000; 
	private final int LOWBARHEIGHT = -62000; 
	private final int LOWBARFORWARDHEIGHT = -60000; 
	private final int SHOOTHEIGHT = -54000;

	//for the sendable chooser
	String defense; 
	SendableChooser chooser;
//TODO START Camera in autoinit
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
		joystick = new Joystick(0);  
		xboxCtr = new Joystick(1); 

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
		chooser.addObject("Low Bar No Shoot", "lowbarNS");
		SmartDashboard.putData("Autonomous Defense Chooser", chooser);
		//TODO Add robot width for spybot to adjust auto

		//sets up variables for PID with the gyro 
		double pGain = 0.9;
		double iGain = 0.00521135; 
		double dGain = 38.834084496; 
		PIDSource gType = new GyroPIDSource ();
		gType.setPIDSourceType(PIDSourceType.kDisplacement);
		gPid = new PIDController(pGain, iGain, dGain, gType, new GyroPIDOutput(), 0.001); //the lowest possible period is 0.001
		gPid.setInputRange(-360, 360);
	}//End robotInit

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit()  
	{    	

		System.out.println("We are in autonomousInit");
		gyro.reset();
		System.out.println("We have reset gyro"); 

		autoStep = 0; // 

		//chooses which auto to run
		defense = chooser.getSelected().toString();
		if (defense.equals("lowbar")) {
			System.out.println("RUNNING LOW BAR");
		} else if (defense.equals("reach")){
			System.out.println("RUNNING REACH");
		} else if (defense.equals("lowbarNS")) {
			System.out.println("RUNNING LOW BAR, NO SHOOT");
		}


		startCalibration(); //starts to bring up the ball intake arm
		autoTurnTo = 55;
		autoDistanceTo = 61; 
		
	}




	/**
	 * This function is called periodically during autonomous
	 */

	int counter =0; 

	public void autonomousPeriodic() 
	{
		if (counter++%20 ==0 )
			System.out.println("Ultrasonic: " + (inchFromHRLV(ultrasonic.getVoltage())) + "\tGyro: " + gyro.getAngle());

		//TODO - we think getaverage will avoid erroneous spikes.  Only case
		// 2 of lowbar used to use average, though.  all others just used get
		double ultraDistance = inchFromHRLV(ultrasonic.getAverageVoltage()); //gets sonar reading

		//calibration 
		if (isCalibrating)
		{
			checkCalibrationStatus(); 
			return; 
		}


		if (defense.equals("lowbar") || defense.equals("lowbarNS")) 
		{
			//TODO: Consider faster speeds for auto, but only if accuracy is high
			//TODO: Consider making speeds, angles, times, distances into static variables
			switch (autoStep)
			{
			case 0: //start the PID to go straight
			{
				//TODO can we go faster?
				tPower = 0.3;
				gPid.setSetpoint(0);
				gPid.enable();
				autoStep = 1; 
				System.out.println("going to 1");
				adjustArmHeight(LOWBARFORWARDHEIGHT); 
				break; 
			}

			case 1: //Continue until 2' from flaps
			{

				if (ultraDistance < 24) //sensor has seen the low bar flaps
				{
					autoStep = 2; 
					System.out.println("going to 2");
				}
				break;
			}

			case 2: //Continue until past flaps
			{ 

				if (ultraDistance > 120) 
				{
					autoTimer.reset();
					autoTimer.start();
					System.out.println("going to 3");
					autoStep = 3; 
				}

				break;

			}

			case 3: //Make sure we really passed flaps
			{
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

				if (ultraDistance < (autoDistanceTo + 24)) //used to be 62, was 106 after Peoria
				{
					gPid.reset(); //changed to reset
					myRobot.drive(0,  0); //added the -0.01 power
					autoStep = 45; 
					tPower = 0.175; //slows the robot down so it won't over shoot as much
					gPid.enable();
					System.out.println(ultraDistance); 
					System.out.println("going to 45");
				}

				break;
			}

			case 45: 
			{
				//this distance still not correct - reason we missed the goal at CIR
				if (ultraDistance < autoDistanceTo) //was 94 after Peoria
				{
					gPid.reset();
					myRobot.drive(0, 0);
					autoStep = 50; 						
				}
				break;
			}

			case 50: //uses a PID to go backwards
			{
				tPower = -0.175; //sets a negative power
				gPid.enable();
				System.out.println("going to 6");
				autoStep = 60;  
				break;
			}

			case 60: // goes backwards until back at the ultrasonic threshold 
			{

				//this distance still not correct - reason we missed the goal at CIR
				if (ultraDistance >= autoDistanceTo)  //was 94 at end of Peoria
				{
					System.out.println(ultraDistance);
					gPid.disable();
					myRobot.setLeftRightMotorOutputs(0,0);
					autoStep = 70;
					//TODO - Consider going to higher shooting height
					adjustArmHeight(SHOOTHEIGHT); //intake put to the right position to shoot
					System.out.println("going to 7");
				}
				break;
			}

			case 70: 
			{

				if (gyro.getAngle()< autoTurnTo) //this angle looked good at CIR
					myRobot.setLeftRightMotorOutputs(-0.3,0.3);

				else 
				{
					autoStep = 80; 
					System.out.println("going to 8");
					myRobot.setLeftRightMotorOutputs(0,0);
				}

				break;
			}

			case 80: // turns slowly back in case of overshoot 
			{
				if (gyro.getAngle() >= autoTurnTo) 
					myRobot.setLeftRightMotorOutputs(0.17,-0.17);
				else 
				{
					autoStep = 90; 
					System.out.println("going to 9");
				}

				break;
			}

			case 90: //uses a PID to start to go forward
			{
				myRobot.drive(0, 0);
				gPid.setSetpoint(autoTurnTo); //test this angle
				tPower = 0.3; 
				gPid.enable();

				autoTimer.reset();
				autoTimer.start();
				autoStep=110; 
				System.out.println("going to 11");

				break;
			}

			case 110: //drive forward for time
			{

				if (autoTimer.get() > 2) //test this num
				{
					gPid.reset();
					myRobot.setLeftRightMotorOutputs(0,0);
					System.out.println("going to 12");
					autoStep = 120; 
				}

				break;
			}

			case 120: //shoot
			{
				if (!defense.equals("lowbarNS")) {
					intake.setLeftRightMotorOutputs(-1, -1);
				}

				if (autoTimer.get()>3)  
				{
					autoStep = 130;
					System.out.println("going to 13");

				}

				break;
			}

			case 130: 
			{
				myRobot.setLeftRightMotorOutputs(0,0);
				intake.drive(0, 0);
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

		//This was inserted because there were times when robot kept driving
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

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() 
	{
		//this line was sometimes was uncommented in a match but should always be commented out unless troubleshooting
		//System.out.println("UltrasonicHRLV: " + (inchFromHRLV(ultrasonic.getVoltage())) + "  Gyro: " + gyro.getAngle());

		//calibrates the arm (raises it up until the hard stop) 
		if (isCalibrating)
			checkCalibrationStatus(); 

		//for turning 180 degrees
		if (isTurning)
		{
			checkTurningStatus();  
		}

		//this line was also uncommented at CIR - should be commented out 
		//System.out.println(currentPosition);

		//TODO - Potentially Slow down turn 
		//get the value of the slider on the joystick 
		//this is used so the driver can adjust max speed anywhere from 60% to 100% 
		double scale = joystick.getRawAxis(3)*-1; 
		scale = ((scale+1)/5)+0.6; 

		//gets the x and y axis values from the joystick 
		moveValue = joystick.getRawAxis(1);
		rotateValue = joystick.getRawAxis(0); 

		//dead zone on y axis value
		if (Math.abs(moveValue)<0.005)
			moveValue = 0; 

		//creates a dead zone on x axis value only if the y axis value is small 
		if (Math.abs(rotateValue)<0.005 && Math.abs(moveValue)<0.1)
			rotateValue = 0;

		//scale down the values 
		moveValue = moveValue*scale; 
		rotateValue = rotateValue*scale; 
		
		/*if (Math.abs(rotateValue) < 0.4)
			rotateValue = rotateValue*scale;*/ 
		

		//if driver tries to turn the robot, stop running the 180 degree turn method
		if (rotateValue > 0)
			isTurning = false; 


		if (!isTurning) //makes sure the driver isn't trying to use the 180 degree turn method
			myRobot.arcadeDrive(moveValue, rotateValue, true); 

		//large dead zone for the bag motors - that button on the xbox doesn't always go back to 0 position easily 
		double intakeAxis = xboxCtr.getRawAxis(5); 
		if(Math.abs(intakeAxis)<0.25) 
			intakeAxis=0; 

		//spins the bag motors
		intake.drive(intakeAxis, 0);

		//this code was used so driver could switch between front and back camera 
		//back camera USB broke at CIR so commented out 

//TODO put camera back
		/*		if(joystick.getRawButton(1))
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


    		while(joystick.getRawButton(1));
    	}*/

		//starts the 180 degree turn
		if (!isTurning)
		{
			if(joystick.getRawButton(2))
				startTurning();
			while(joystick.getRawButton(2));
		}

		//provides the driver another way to get out of 180 degree turn method
		if (joystick.getRawButton(5)) // see if Jack likes this button 
			isTurning = false; 

		//camera displayed on dahsboard
		activeCamera.getImage(img);
		server.setImage(img); // puts image on the dashboard

		//allows the drive to invert the motors to help with driving backwards
		if (joystick.getRawButton(3)) //changed 3.14 from button 2 to button 3
		{  	     		

			inverted=!inverted; 

			myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kFrontLeft, inverted);
			myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kRearLeft, inverted);
			myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kFrontRight, inverted);
			myRobot.setInvertedMotor(SICPRobotDrive.MotorType.kRearRight, inverted);
			while (joystick.getRawButton(3));
		}

		//button functions on the xBox 
		//The buttons on the xBox are Y(top, 3) B(right,2) A(bottom, 1) X(left, 4)    
		TeleopFunctions chosen = TeleopFunctions.NONE;     	

		//TODO - adjust buttons per Angela's request
		//Y is for getting the arm to the right place to shoot the ball
		if (xboxCtr.getRawButton(3)==true)
			chosen = TeleopFunctions.SHOOT; 

		//A is for getting the arm to the right place for crossing low bar when ball intake is in backwards
		else if (xboxCtr.getRawButton(1)==true)
			chosen = TeleopFunctions.LOWBARBACKWARD; 

		//Upper Left button is for bringing the intake back up to the hard stop (calibrating it) 
		else if (xboxCtr.getRawButton(5)==true)
			chosen = TeleopFunctions.RESET;

		//B is for getting the arm to the right place to grab a ball  
		else if (xboxCtr.getRawButton(2)==true)
			chosen = TeleopFunctions.GRABBALL;

		//Upper Right Button is for holding the intake arm in its place
		else if (xboxCtr.getRawButton(6)==true)
			chosen = TeleopFunctions.HOLD;

		//X is for getting the arm to the right place for crossing low bar when ball intake is in front
		else if (xboxCtr.getRawButton(4)== true)
			chosen = TeleopFunctions.LOWBARFORWARDS; 

		//If the button is held down, don't repeat the position set
		if (chosen != lastChosen) {

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
		}
		lastChosen = chosen;

		//allows arm to also be manually controlled 
		if(!isCalibrating&&chosen!=TeleopFunctions.HOLD) {
			double armAxis = xboxCtr.getRawAxis(1); 

			if(armR.getControlMode()==TalonControlMode.Position)
			{
				System.out.println("EXITING POSITION MODE");
//TODO : Does the arm dead zone need to be bigger to allow positions to hold (PID)
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
//TODO - check for xbox controller type to have no user mess up


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
		//TODO this was .6 until inspection.  raise back to clear bumpers if necessary
		armR.set(0.35);

	}

	public boolean checkCalibrationStatus()
	{
		double timerVal = calTimer.get();
		/*    	System.out.println("calTimer:" + timerVal);*/

		//calibration will run for at least 0.1 seconds
		//TODO this should be longer. only needed to clear bumpers
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

			armR.set(0);
			armR.setPosition(0); 
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
				myRobot.setLeftRightMotorOutputs(0, 0);
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

//TODO: Use a PID for the 180

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
}
