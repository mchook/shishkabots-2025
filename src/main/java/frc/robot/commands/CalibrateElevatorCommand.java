package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.Logger;

/**
 * Command to calibrate the elevator's zero position.
 * This should be used when the elevator is physically at its lowest position.
 */
public class CalibrateElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private boolean isFinished = false;
    
    /**
     * Creates a new CalibrateElevatorCommand.
     * 
     * @param elevatorSubsystem The elevator subsystem to calibrate
     */
    public CalibrateElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        elevator = elevatorSubsystem;
        addRequirements(elevator);
    }
    
    @Override
    public void initialize() {
        Logger.log("Starting elevator zero position calibration");
    }
    
    @Override
    public void execute() {
        // First stop the elevator to ensure it's not moving
        elevator.stop();
        
        // Calibrate the zero position
        double previousPosition = elevator.calibrateZeroPosition();
        
        Logger.log("Elevator calibrated. Position adjusted from " + previousPosition + " to 0.0");
        
        // Mark as finished after calibration
        isFinished = true;
    }
    
    @Override
    public boolean isFinished() {
        return isFinished;
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Logger.log("Elevator calibration was interrupted!");
        } else {
            Logger.log("Elevator calibration completed successfully");
        }
    }
}
