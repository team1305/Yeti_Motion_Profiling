// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Subsystem_Intake extends SubsystemBase {
  /** Creates a new Subsystem_Intake. */


  private final static WPI_TalonFX mtIntake = Constants.mtIntake;


private final static Solenoid slndIntake1 = Constants.slndIntake;

//private final static Solenoid slndIntake2 = Constants.slndIntake2;
  public boolean bintakeOn;

  public Subsystem_Intake() {

   // Trying to get brake mode working
   mtIntake.setNeutralMode(NeutralMode.Brake);

   mtIntake.configStatorCurrentLimit(Constants.currentLimitConfig, 40);
  /* if (RobotContainer.getdebug()) {
      SmartDashboard.putBoolean("Intake Solenoid Set Value", false);
   }*/
   bintakeOn = false;
   //intakeExtension(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntake(double speed) {
    //mtIntake.set(ControlMode.PercentOutput, speed);
    mtIntake.set(speed);
    //slndIntake1.set(true);  //added bjk at FLL
    //SmartDashboard.putString("Intake Audit", "setIntake");
    //bintakeOn = true;
    //SmartDashboard.putBoolean("Intake Solenoid Set Value", true);
  }


  public void stopIntake() {
    mtIntake.set(0);
    //slndIntake1.set(false);  //added bjk at fll
    //SmartDashboard.putBoolean("Intake Solenoid Set Value", false);
    //bintakeOn = false;
  }


  public void intakeExtension(boolean extension) {
    
    
    if (extension){
       slndIntake1.set(true);
       bintakeOn = true;
       /*if (RobotContainer.getdebug()) {
          SmartDashboard.putBoolean("Intake Solenoid Set Value", true);
          SmartDashboard.putString("Intake Audit", "intakeextension");
       }*/
    } else {
      slndIntake1.set(false);
      bintakeOn = false;
      /*if (RobotContainer.getdebug()) {
        SmartDashboard.putBoolean("Intake Solenoid Set Value", false);  
      }*/
    }
    
  }


  public boolean isIntakeOn(){
    if (bintakeOn) {
      return true;
    }else {
      return false;
    }
    //return bintakeOn;
    
  }


  public void Intake_Toggle(){
    if(bintakeOn){
      //if (RobotContainer.getdebug()) {
      //  SmartDashboard.putString("Intake Toggle", "Off"); 
      //}
      intakeExtension(false);
       setIntake(0);
    }else{
      //if (RobotContainer.getdebug()) {
      //  SmartDashboard.putString("Intake Toggle", "On");
      //}
      intakeExtension(true);
      setIntake(0.5);
    }



  }










}
