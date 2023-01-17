// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Subsystem_Compressor extends SubsystemBase {
  /** Creates a new Subsystem_Compressor_Power. */
  private final Compressor compressor = Constants.cmpRobotCompressor;

  //turns the compressor on
  public void CompressorON() {

    compressor.enableDigital();
    
  }

  //checks to see if the compressor is enabled
  public boolean isenabled() {
    return compressor.enabled();
  }

  //turns the compressor off
  public void CompressorOFF() {

    compressor.disable();
  }


  public Subsystem_Compressor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
