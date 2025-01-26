package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** 
 * An example autonomous command that drives the robot forward for a specified duration.
 */
public class AutonomousCommand extends Command {
    private final DriveSubsystem m_drive;
    private final double m_duration;
    private long m_startTime;

    public AutonomousCommand(DriveSubsystem driveSubsystem, double durationSeconds) {
        m_drive = driveSubsystem;
        m_duration = durationSeconds;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        // Drive forward at 50% speed
        m_drive.drive(0.5, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        // Command finishes when the duration has elapsed
        return (System.currentTimeMillis() - m_startTime) >= (m_duration * 1000);
    }
}
