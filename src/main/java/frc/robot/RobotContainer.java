// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.custom.Deadbander;
import frc.robot.subsystems.drive.Drivetrain;

public class RobotContainer {

  private Drivetrain mDrivetrain = new Drivetrain();
  private CommandXboxController mController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    mDrivetrain.setDefaultCommand(
      new RunCommand(
        () -> mDrivetrain.drive(
          -Deadbander.applyLinearScaledDeadband(mController.getLeftY() * 4, 0.1),
          -Deadbander.applyLinearScaledDeadband(mController.getLeftX() * 4, 0.1),
          -Deadbander.applyLinearScaledDeadband(mController.getRightX() * 4, 0.1),
          true
        ), 
        mDrivetrain
      )
    );

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
