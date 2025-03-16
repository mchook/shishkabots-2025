package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax primaryElevatorMotor;
    private final SparkMax secondaryElevatorMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;
    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    // Simulation
    private final DCMotor elevatorDCMotor;
    private final SparkMaxSim primaryElevatorMotorSim;
    private final SparkMaxSim secondaryElevatorMotorSim;

    // Constants
    private static final double MAX_OUTPUT = 1.0;
    private static final double MIN_OUTPUT = -1.0;
    private static final double TOLERANCE = 0.5;

    // Elevator Position Constants (in encoder units)
    private static final double BOTTOM_THRESHOLD = -5.0;
    private static final double TOP_THRESHOLD = 40.0;  // Adjust based on actual max height
    private static final double LEVEL_1_HEIGHT = 10.0;  // Ground/Bottom level
    private static final double LEVEL_2_HEIGHT = 20.0;  // Mid level
    private static final double LEVEL_3_HEIGHT = 30.0;  // Top level

    // PID Constants - Tune these values during testing
    private static final double kP = 0.02;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kFF = 0.0;

    private static final int MAX_CURRENT = 40;
    // Position Control
    private double targetPosition = 0.0;

    // Periodic counter for status updates
    private int periodicCounter = 0;

    /*
     * Elevator max height = 63 inches
     * First level = 29 inches
     * Second level = 44.5 inches
     * Third level = 70 inches
     */

    public ElevatorSubsystem(int primaryMotorCanId, int secondaryMotorCanId, int topLimitSwitchId, int bottomLimitSwitchId) {
        // Initialize motors
        primaryElevatorMotor = new SparkMax(primaryMotorCanId, SparkMax.MotorType.kBrushless);
        secondaryElevatorMotor = new SparkMax(secondaryMotorCanId, SparkMax.MotorType.kBrushless);
        
        // Initialize limit switches
        topLimitSwitch = new DigitalInput(topLimitSwitchId);
        bottomLimitSwitch = new DigitalInput(bottomLimitSwitchId);
        
        // Get encoder and controller from primary motor
        encoder = primaryElevatorMotor.getEncoder();
        closedLoopController = primaryElevatorMotor.getClosedLoopController();

        // Configure the primary motor with PID
        SparkMaxConfig primaryConfig = new SparkMaxConfig();
        primaryConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MAX_CURRENT);
        
        primaryConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .velocityFF(kFF)
            .outputRange(MIN_OUTPUT, MAX_OUTPUT);
            
        primaryElevatorMotor.configure(
            primaryConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        
        // Configure the secondary motor (follower)
        SparkMaxConfig secondaryConfig = new SparkMaxConfig();
        secondaryConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MAX_CURRENT);
        
        secondaryElevatorMotor.configure(
            secondaryConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        
        // Reset encoder position
        resetEncoder();
        
        // Initialize simulation objects if in simulation mode
        if (RobotBase.isSimulation()) {
            elevatorDCMotor = DCMotor.getNEO(1);
            primaryElevatorMotorSim = new SparkMaxSim(primaryElevatorMotor, elevatorDCMotor);
            secondaryElevatorMotorSim = new SparkMaxSim(secondaryElevatorMotor, elevatorDCMotor);
        } else {
            elevatorDCMotor = null;
            primaryElevatorMotorSim = null;
            secondaryElevatorMotorSim = null;
        }
        
        // Log initialization
        System.out.println("Elevator subsystem initialized");
    }
    
    /**
     * Set the target position for the elevator
     * @param position Target position in encoder units
     */
    public void setTargetPosition(double position) {
        // Safety check to ensure position is within bounds
        position = Math.min(Math.max(position, BOTTOM_THRESHOLD), TOP_THRESHOLD);
        
        targetPosition = position;
        System.out.println("Setting elevator position to " + position + " (current: " + getCurrentPosition() + ")");
        
        // Use the PID controller to move to the position
        closedLoopController.setReference(position, ControlType.kPosition);
    }
    
    public double getCurrentPosition() {
        return encoder.getPosition();
    }
    
    public boolean atTargetPosition() {
        return Math.abs(getCurrentPosition() - targetPosition) < TOLERANCE;
    }
    
    public boolean isAtTop() {
        return !topLimitSwitch.get();  // Limit switches are typically active LOW
    }
    
    public boolean isAtBottom() {
        return !bottomLimitSwitch.get();  // Limit switches are typically active LOW
    }
    
    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public void stop() {
        System.out.println("***** Stopping elevator at position: " + getCurrentPosition());
        primaryElevatorMotor.stopMotor();
        secondaryElevatorMotor.stopMotor();  // Stop both motors
    }

    /**
     * Sets the elevator to a predefined level
     * @param level 1 for bottom, 2 for middle, 3 for top
     */
    public void goToLevel(int level) {
        System.out.println("Moving elevator to level " + level);
        switch (level) {
            case 1:
                setTargetPosition(LEVEL_1_HEIGHT);
                break;
            case 2:
                setTargetPosition(LEVEL_2_HEIGHT);
                break;
            case 3:
                setTargetPosition(LEVEL_3_HEIGHT);
                break;
            default:
                throw new IllegalArgumentException("Invalid level: " + level);
        }
    }

    /**
     * Returns the current level of the elevator (1, 2, or 3)
     * Returns 0 if between levels
     */
    public int getCurrentLevel() {
        double position = getCurrentPosition();
        if (position < LEVEL_1_HEIGHT + TOLERANCE) {
            return 1;
        } else if (position < LEVEL_2_HEIGHT + TOLERANCE) {
            return 2;
        } else {
            return 3;
        }
    }

    @Override
    public void periodic() {
        // Safety checks - stop if either limit switch is triggered OR position exceeds thresholds
        if (isAtTop() || getCurrentPosition() > TOP_THRESHOLD) {
            if (primaryElevatorMotor.get() > 0) {
                System.out.println("Elevator at top limit or exceeded threshold - STOPPING");
                stop();
            }
        }
        
        if (isAtBottom() || getCurrentPosition() < BOTTOM_THRESHOLD) {
            if (primaryElevatorMotor.get() < 0) {
                System.out.println("Elevator at bottom limit or exceeded threshold - STOPPING");
                stop();
            }
        }
        
        // Print periodic status every 50 calls (about once per second)
        if (periodicCounter++ % 50 == 0) {
            System.out.println(String.format("Elevator Status - Pos: %.2f, Target: %.2f, P1 Speed: %.2f, P2 Speed: %.2f",
                getCurrentPosition(), targetPosition, 
                primaryElevatorMotor.get(), secondaryElevatorMotor.get()));
        }
        
        updateTelemetry();
        
        // Update simulation
        if (RobotBase.isSimulation()) {
            updateSimulatorState();
        }
    }

    public void updateSimulatorState() {
        double positionError = targetPosition - encoder.getPosition();
        double velocityInchPerSec = positionError / 0.02;  // Basic simulation
        primaryElevatorMotorSim.iterate(velocityInchPerSec, primaryElevatorMotor.getBusVoltage(), 0.02);
        secondaryElevatorMotorSim.iterate(velocityInchPerSec, secondaryElevatorMotor.getBusVoltage(), 0.02);
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator/CurrentPosition", getCurrentPosition());
        SmartDashboard.putNumber("Elevator/TargetPosition", targetPosition);
        SmartDashboard.putNumber("Elevator/CurrentLevel", getCurrentLevel());
        SmartDashboard.putBoolean("Elevator/AtTop", isAtTop());
        SmartDashboard.putBoolean("Elevator/AtBottom", isAtBottom());
        SmartDashboard.putBoolean("Elevator/AtTarget", atTargetPosition());
        
        SmartDashboard.putNumber("Elevator/Primary/Current", primaryElevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/Primary/Voltage", primaryElevatorMotor.getBusVoltage());
        SmartDashboard.putNumber("Elevator/Primary/Speed", primaryElevatorMotor.get());
        
        SmartDashboard.putNumber("Elevator/Secondary/Current", secondaryElevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/Secondary/Voltage", secondaryElevatorMotor.getBusVoltage());
        SmartDashboard.putNumber("Elevator/Secondary/Speed", secondaryElevatorMotor.get());
    }
}
