package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightDebugCommand extends Command {
    private final LimelightSubsystem m_limelight;

    public LimelightDebugCommand(LimelightSubsystem limelight) {
        m_limelight = limelight;
        addRequirements(limelight);
    }

    @Override
    public void execute() {
        // Get all values from Limelight
        boolean hasTarget = m_limelight.hasValidTarget();
        double tx = m_limelight.getX();
        double ty = m_limelight.getY();
        double ta = m_limelight.getArea();

        // Print to SmartDashboard
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget);
        SmartDashboard.putNumber("Limelight X", tx);
        SmartDashboard.putNumber("Limelight Y", ty);
        SmartDashboard.putNumber("Limelight Area", ta);
        
        // Add raw values for debugging
        SmartDashboard.putNumber("Limelight distance", m_limelight.getDistanceFromTag(1.6, -m_limelight.getX()));
        SmartDashboard.putNumber("Limelight Raw TV", m_limelight.getRawTargetValue());
        SmartDashboard.putNumber("Limelight Target ID", m_limelight.getTargetID());
    }

    @Override
    public boolean isFinished() {
        // This command never finishes on its own - it runs until interrupted
        return false;
    }
}
