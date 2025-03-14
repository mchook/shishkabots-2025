package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;


public class NearestBranches extends Command {
    private final DriveSubsystem driveSubsystem;
    private final Pose2d[] leftBranchPositions = Constants.Locations.leftBranchLocations;
    private final Pose2d[] rightBranchPositions = Constants.Locations.rightBranchLocations;
    private Pose2d endPose;
    
    public NearestBranches(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
        addRequirements(getRequirements());
    }

    @Override
    public void initialize() {
        Pose2d startingPose = driveSubsystem.getPose();
        //true means drive to right branch, false means drive to left branch - temporary place holder until keybinds are implemented
        driveToNearestBranch(startingPose, true);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        double distance = distanceFormula(driveSubsystem.getPose(), endPose);
        double threshold = 0.2;
        if (distance <= threshold) {
            return true;
        }
        return false;
    }

    private void driveToNearestBranch(Pose2d startingPose, boolean driveToRightBranch) {
        if (driveToRightBranch) {
            endPose = getNearestBranch(startingPose, rightBranchPositions);
        } else {
            endPose = getNearestBranch(startingPose, leftBranchPositions);
        }
        driveSubsystem.driveToEndPose(endPose);
    }
    
    private double distanceFormula(Pose2d startingPose, Pose2d endPose) {
        double x1 = startingPose.getX();
        double y1 = startingPose.getY();
        double x2 = endPose.getX();
        double y2 = endPose.getY();
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    private Pose2d getNearestBranch(Pose2d startingPose, Pose2d[] branchPositions) {
        double minDistance = Double.MAX_VALUE;
        Pose2d nearestBranch = null;
        for (int i = 0; i < branchPositions.length; i++) {
            double distance = distanceFormula(startingPose, branchPositions[i]);
            if (distance < minDistance) {
                minDistance = distance;
                nearestBranch = branchPositions[i];
            }
        }
        return nearestBranch;
    }
}
