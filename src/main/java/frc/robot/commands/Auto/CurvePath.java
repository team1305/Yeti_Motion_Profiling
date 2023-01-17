// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Subsystem_Drivebase_Motion;
import frc.robot.subsystems.Subsystem_Intake;
import frc.robot.subsystems.Subsystem_Drivebase_Motion.AutoPath;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CurvePath extends SequentialCommandGroup {

  Subsystem_Drivebase_Motion subDrivetrain;
  Trajectory trajectory;
  RamseteCommand test;
  
  /** Creates a new CurvePath. */
  public CurvePath(Subsystem_Drivebase_Motion subDrivetrain) {
    this.subDrivetrain = subDrivetrain;
    trajectory = subDrivetrain.getTrajectory(AutoPath.CurvePath);
    test = subDrivetrain.getRamseteCommand(trajectory);
    addCommands(
        new InstantCommand(() -> subDrivetrain.resetOdeometry(trajectory.getInitialPose())),
        test);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }

}
