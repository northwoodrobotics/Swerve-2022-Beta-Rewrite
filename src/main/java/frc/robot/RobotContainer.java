// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
//import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import ExternalLib.SpectrumLib.controllers.SpectrumXboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import ExternalLib.JackInTheBotLib.robot.input.Axis;
import ExternalLib.JackInTheBotLib.robot.input.DPadButton;
import ExternalLib.JackInTheBotLib.robot.input.JackInTheBotXboxController;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.math.trajectory.Trajectory;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final JackInTheBotXboxController driveController = new JackInTheBotXboxController(0);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final Superstructure superstructure = new Superstructure();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    

    CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);

    CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link JackInTheBotXboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   * 
   * 
   */

  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 /* public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
  }*/
  


private Axis getDriveForwardAxis() {
    return driveController.getLeftYAxis();
}

private Axis getDriveStrafeAxis() {
    return driveController.getLeftXAxis();
}

private Axis getDriveRotationAxis() {
    return driveController.getRightXAxis();
}

public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
}

public Superstructure getSuperstructure() {
    return superstructure;
}


}
