package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.LinkedList;
import java.util.Queue;

public class TestAllCoralPos extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final Queue<Pose2d> positionQueue = new LinkedList<>();
    private Command currentCommand;

    public TestAllCoralPos(DriveSubsystem drive) {
        m_driveSubsystem = drive;
        addRequirements(m_driveSubsystem);

        // Load all positions into a queue
        for (Pose2d pos : Constants.Locations.leftBranchLocations) {
            positionQueue.add(pos);
        }
        for (Pose2d pos : Constants.Locations.rightBranchLocations) {
            positionQueue.add(pos);
        }
    }

    @Override
    public void initialize() {
        System.out.println("Starting TestAllCoralPos");
        nextPose();
    }

    @Override
    public void execute() {
        if (currentCommand == null || currentCommand.isFinished()) {
            nextPose();
        }
    }

    private void nextPose() {
        if (!positionQueue.isEmpty()) {
            Pose2d nextPosition = positionQueue.poll();
            System.out.println("Moving to: " + nextPosition);
            
            currentCommand = m_driveSubsystem.driveToEndPose(nextPosition)
                .andThen(new InstantCommand(this::nextPose)); // Chain the next command
            
            currentCommand.schedule();
        } else {
            currentCommand = null;
        }
    }

    @Override
    public boolean isFinished() {
        boolean done = positionQueue.isEmpty() && (currentCommand == null || currentCommand.isFinished());
        if (done) System.out.println("TestAllCoralPos is fully complete!");
        return done;
    }
}