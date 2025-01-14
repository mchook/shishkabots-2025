// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  // The driver's controller
  private final XboxController driverController = new XboxController(0);

  public RobotContainer() {
    configureBindings();

    // Set up the default command for the drive subsystem
    driveSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            driveSubsystem,
            () -> -driverController.getLeftY() * 0.5,  // Forward/backward
            () -> -driverController.getLeftX() * 0.5,  // Left/right
            () -> -driverController.getRightX() * 0.5  // Rotation
        )
    );
  }

  private void configureBindings() {
    // Add button bindings here
    // Stop the robot when the B button is pressed
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .onTrue(Commands.runOnce(() -> driveSubsystem.stop()));
    
    // Half speed mode while holding right bumper
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(
            new DefaultDriveCommand(
                driveSubsystem,
                () -> -driverController.getLeftY() * 0.25,  // Half speed forward/backward
                () -> -driverController.getLeftX() * 0.25,  // Half speed left/right
                () -> -driverController.getRightX() * 0.25   // Half speed rotation
            )
        );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
