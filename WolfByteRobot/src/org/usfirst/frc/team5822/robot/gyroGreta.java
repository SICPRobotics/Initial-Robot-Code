package org.usfirst.frc.team5822.robot;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

public class gyroGreta extends GyroBase
{

	
	double angle = 0; 
	SPI spi = new SPI(Port.kOnboardCS0);

	public gyroGreta()
	{ 
		
		spi.setClockRate(4000000); //4 MHz (rRIO max, gyro can go high) 
		spi.setClockActiveHigh();
		spi.setChipSelectActiveLow();
		spi.setMSBFirst();
	}
	
	public void calibrate() 
	{
		
		
	}


	public void reset() 
	{
		angle = 0; 
		
	}

	
	public double getAngle() 
	{
	
		return angle;
	}

	
	public double getRate() 
	{
		
		return 0;
	}

}
