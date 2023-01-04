// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    
    //Motor ID for the drive base
	public static WPI_TalonFX mtDriveRight2 = new WPI_TalonFX(11);
	public static WPI_TalonFX mtDriveRight1 = new WPI_TalonFX(12);
	public static WPI_TalonFX mtDriveLeft1 = new WPI_TalonFX(13);
	public static WPI_TalonFX mtDriveLeft2 = new WPI_TalonFX(14);
    public final static StatorCurrentLimitConfiguration currentLimitConfig = new StatorCurrentLimitConfiguration(true, 40, 39.95, 1);
	public final static StatorCurrentLimitConfiguration currentLimitConfig30 = new StatorCurrentLimitConfiguration(true, 30, 29.95, 1);

	//public static final int[] kLeftEncoderPorts = new int[] {13, 14};
	//public static final int[] kRightEncoderPorts = new int[] {11, 12};
	//public static final boolean kLeftEncoderReversed = false;
	//public static final boolean kRightEncoderReversed = false;

	public static final int kEncoderCPR = 2048;
	public static final double kWheelDiameterMeters = 0.1524;// REMEBER TO PUT MEASUREMENTS IN METERS
	public static final double kGearBoxReduction = 10.71;
	public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR / kGearBoxReduction;
	//public static final double kEncoderDistancePerPulse = 512;

    public static final double ksVolts = 0.53111;
    public static final double kvVoltSecondsPerMeter = 2.4475;
    public static final double kaVoltSecondsSquaredPerMeter = 0.21498;
    public static final double kPDriveVel = 2.9309;
	
	

	public static final double kTrackwidthMeters = Units.inchesToMeters(20);//CHANGE THIS TO BE ACTUAL WIDTH
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

		
	public static final double kMaxSpeedMetersPerSecond = 1;//set low test
	public static final double kMaxAccelerationMetersPerSecondSquared = 1;//set low to test

	    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;




}
