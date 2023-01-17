// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Subsystem_Intake;


public class Command_Intake extends CommandBase {
  /** Creates a new Command_Intake. */
  private final Subsystem_Intake intakeSub;
  //private final Subsystem_LED ledSub;
 

  
  public Command_Intake(Subsystem_Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSub = intake; 
    //ledSub = led;
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intake.intakeExtension(true);
    RobotContainer.intake.setIntake(0.75); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.intakeExtension(false);
    RobotContainer.intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
