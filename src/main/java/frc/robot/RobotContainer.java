// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Auto.IntakeTest;
import frc.robot.commands.Auto.StraightPath;
import frc.robot.commands.Auto.CurvePath;
import frc.robot.subsystems.Subsystem_Compressor;
//import frc.robot.commands.ExampleCommand;
//import frc.robot.subsystems.ExampleSubsystem;
//import frc.robot.subsystems.Subsystem_Drivebase;
import frc.robot.subsystems.Subsystem_Drivebase_Motion;
import frc.robot.subsystems.Subsystem_Intake;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Xbox Controllers
    final static XboxController PRIMARY = new XboxController(0);

    //public final static Subsystem_Drivebase drive = new Subsystem_Drivebase();
    public final static Subsystem_Drivebase_Motion subDrivetrain = new Subsystem_Drivebase_Motion();

    private final StraightPath testAuto = new StraightPath(subDrivetrain);
    private final IntakeTest intakeTest = new IntakeTest(subDrivetrain, intake);
    private final CurvePath CurvePath = new CurvePath(subDrivetrain);

    public static Subsystem_Compressor compressor = new Subsystem_Compressor();
    public final static Subsystem_Intake intake = new Subsystem_Intake();
    
    SendableChooser<Command> autoChooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutoSelector();
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 
  private void configureAutoSelector() {
    autoChooser.addOption("null", null);
    autoChooser.setDefaultOption("Straight", testAuto);
    autoChooser.setDefaultOption("Intake test", intakeTest);
    autoChooser.setDefaultOption("Curved path", CurvePath);
    //autoChooser.addOption("LEFT_FENDER_POSITION_FRONT", new InstantCommand(() -> subDrivetrain.resetPose(constField.LEFT_FENDER_POSITION_FRONT)));
    SmartDashboard.putData(autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

   // returns joyxbox1 whenever getJoystickDriver is called
   public static XboxController getJoystickDriver() {
    return PRIMARY;
  }
}
