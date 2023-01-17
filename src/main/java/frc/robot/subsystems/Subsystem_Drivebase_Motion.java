// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.examples.ramsetecommand.Constants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Auto.StraightPath;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Subsystem_Drivebase_Motion extends SubsystemBase {
  AHRS navx;
  //private boolean bteleop;

    // grabs drive motor information from RobotMap
    private final static WPI_TalonFX mtLeft1 = Constants.mtDriveLeft1;
    private final static WPI_TalonFX mtLeft2 = Constants.mtDriveLeft2;
    private final static WPI_TalonFX mtRight1 = Constants.mtDriveRight1;
    private final static WPI_TalonFX mtRight2 = Constants.mtDriveRight2;

        // creates motor controller groups for left and right motors
        final  MotorControllerGroup m_leftMotors = new MotorControllerGroup(mtLeft1, mtLeft2);
        final  MotorControllerGroup m_rightMotors = new MotorControllerGroup(mtRight1, mtRight2);
    


  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The gyro sensor
  public final Gyro m_gyro = new AHRS(SPI.Port.kMXP);
 


  // Odometry class for tracking robot pose
  private static DifferentialDriveOdometry m_odometry;

  Trajectory TRAJ_StraightPath;
  Trajectory TRAJ_IntakeTest;
  Trajectory TRAJ_CurvePath;

  /** Creates a new DriveSubsystem. */
  public Subsystem_Drivebase_Motion() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_gyro.reset();


    mtLeft1.configFactoryDefault();
    mtLeft2.configFactoryDefault();
    mtRight1.configFactoryDefault();
    mtRight2.configFactoryDefault();

    mtLeft1.setNeutralMode(NeutralMode.Brake);
    mtLeft2.setNeutralMode(NeutralMode.Brake);
    mtRight1.setNeutralMode(NeutralMode.Brake);
    mtRight2.setNeutralMode(NeutralMode.Brake);

    m_leftMotors.setInverted(false); // this makes the robot spin really fast
    m_rightMotors.setInverted(true);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    loadTrajectories();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    //if (bteleop = false) {
       m_odometry.update(
        m_gyro.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
        SmartDashboard.putNumber("Ratio", getGearRatio());
       SmartDashboard.putNumber("Right Encoder (Ticks)", mtRight1.getSelectedSensorPosition());
       SmartDashboard.putNumber("Left Encoder (Ticks)", mtLeft1.getSelectedSensorPosition());
       SmartDashboard.putNumber("Right Encoder Distance (Meteres)", getRightEncoderDistance());
       SmartDashboard.putNumber("Left Encoder Distance (Meteres)", getLeftEncoderDistance());
       SmartDashboard.putNumber("Right Encoder Speed", getRightEncoderSpeed());
       SmartDashboard.putNumber("Left Encoder Speed", getLeftEncoderSpeed());
       SmartDashboard.putNumber("GYRO", m_gyro.getAngle());
       SmartDashboard.putNumber("AVERAGE DISTANCE", getAverageEncoderDistance());
    //} 
  }


  private double getGearRatio() {
    //return (50.0/14.0)*(48.0/16.0);
    return (50.0/14.0)*(50.0/14.0);
  }

  public double nativeUnitsToDistanceMeters(double sensorCounts){
    double kCountsPerRev = 2048.0;
    double kWheelDiameterInches = 6.0;
		double motorRotations = sensorCounts / kCountsPerRev;
		double wheelRotations = motorRotations / getGearRatio();
		double positionMeters = wheelRotations * (Math.PI * Units.inchesToMeters(kWheelDiameterInches));
    SmartDashboard.putNumber("Motor Rotations", motorRotations);
    SmartDashboard.putNumber("Wheel Rotations", wheelRotations);
    SmartDashboard.putNumber("positionMeters", positionMeters);
		return positionMeters;
	}

  public double nativeUnitsToVelocityMetersPerSecond(double sensorCountsPer100ms){
    double k100msPerSecond = 10.0;
    double velocityMetersPerSecond = nativeUnitsToDistanceMeters(sensorCountsPer100ms) * k100msPerSecond;
		return velocityMetersPerSecond;
	}
  public double getLeftEncoderDistance() {
    return nativeUnitsToDistanceMeters(mtLeft1.getSelectedSensorPosition());
  }

  public double getRightEncoderDistance() {
    return -nativeUnitsToDistanceMeters(mtRight1.getSelectedSensorPosition());
    // negative sign was added
  }

  public double getLeftEncoderSpeed() {
    return nativeUnitsToVelocityMetersPerSecond(mtLeft1.getSelectedSensorVelocity());
  }

  public double getRightEncoderSpeed() {
    return -nativeUnitsToVelocityMetersPerSecond(mtRight1.getSelectedSensorVelocity());
    // negative sign was added
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderSpeed(), getRightEncoderSpeed());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdeometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    mtLeft1.setSelectedSensorPosition(0);
    mtRight1.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public void loadTrajectories() {
    try {
      TRAJ_StraightPath = TrajectoryUtil.fromPathweaverJson(Constants.PATH_StraightPath);
      TRAJ_IntakeTest = TrajectoryUtil.fromPathweaverJson(Constants.PATH_TestPath);
      TRAJ_CurvePath = TrajectoryUtil.fromPathweaverJson(Constants.PATH_TestPath);
      //StraightPath = TrajectoryUtil.fromPathweaverJson("C:\\Users\\1305\\Desktop\\Robot Code repository\\Yeti_motion_profiling\\src\\main\\deploy\\deploy\\pathplanner\\generatedJSON\\StraightPath.wpilib.json");

    } catch (Exception exception) {
      DriverStation.reportError("Unable to open trajectory: ", exception.getStackTrace());
      System.out.println("Unable to read trajectory" + Constants.PATH_StraightPath);

    }
  }

  public enum AutoPath {
    StraightPath,
    IntakeTest,
    CurvePath
  }

  public Trajectory getTrajectory(AutoPath trajectory) {
    switch (trajectory) {

      case StraightPath:
        return TRAJ_StraightPath;
      case IntakeTest:
        return TRAJ_IntakeTest;
        case CurvePath:
          return TRAJ_CurvePath;

      default:
        System.out.println("RETURN NULL");
        return null;
        
    }
  }

  public RamseteCommand getRamseteCommand(Trajectory trajectory) {

    return new RamseteCommand(
      /*
      trajectory,
      this::getPose,
      new RamseteController(),
      Constants.kDriveKinematics,
      this::tankDriveVolts,
      this);
  */
      trajectory,
      this::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(
          Constants.ksVolts,
          Constants.kvVoltSecondsPerMeter,
          Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      this::getWheelSpeeds,
      new PIDController(Constants.kPDriveVel, 0, 0),
      new PIDController(Constants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      this::tankDriveVolts,
      this);
  }

}