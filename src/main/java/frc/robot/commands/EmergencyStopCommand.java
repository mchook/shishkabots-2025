package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Emergency stop command that stops all subsystems immediately
 */
public class EmergencyStopCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    public EmergencyStopCommand(
            DriveSubsystem driveSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ShooterSubsystem shooterSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        
        // Add all subsystems as requirements to ensure no other commands run on them
        addRequirements(driveSubsystem, elevatorSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
        Logger.log("EMERGENCY STOP ACTIVATED");
        
        // Stop all subsystems
        driveSubsystem.stop();
        elevatorSubsystem.stop();
        shooterSubsystem.emergencyStop();
        
        // Add visual indicator to dashboard
        SmartDashboard.putBoolean("EmergencyStop", true);
    }

    @Override
    public boolean isFinished() {
        // This is a one-shot command that finishes immediately after stopping everything
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.log("Emergency stop complete");
    }
}
