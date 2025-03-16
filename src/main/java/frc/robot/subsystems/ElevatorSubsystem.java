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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax primaryElevatorMotor;
    private final SparkMax secondaryElevatorMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;
    
    // Limit switches
    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;
    
    // Simulation objects
    private SparkMaxSim primaryElevatorMotorSim;
    private SparkMaxSim secondaryElevatorMotorSim;
    private DIOSim topLimitSwitchSim;
    private DIOSim bottomLimitSwitchSim;
    private final DCMotor elevatorDCMotor = DCMotor.getNEO(1);
    
    // Constants
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

    // Soft start parameters
    private static final double INITIAL_POWER = 0.3; // Initial power for soft start
    private static final double RAMP_RATE = 0.05; // How quickly to ramp up to full PID control
    private boolean isSoftStartActive = false;
    private double softStartPower = 0.0;
    private double targetPosition = 0.0;
    private Timer softStartTimer = new Timer();

    private static final int MAX_CURRENT = 40;
    
    // Periodic counter for status updates
    private int periodicCounter = 0;
    
    public ElevatorSubsystem(int primaryMotorCanId, int secondaryMotorCanId, 
                           int topLimitSwitchDioChannel, int bottomLimitSwitchDioChannel) {
        // Initialize motors
        primaryElevatorMotor = new SparkMax(primaryMotorCanId, SparkMax.MotorType.kBrushless);
        secondaryElevatorMotor = new SparkMax(secondaryMotorCanId, SparkMax.MotorType.kBrushless);
        
        // Initialize limit switches
        topLimitSwitch = new DigitalInput(topLimitSwitchDioChannel);
        bottomLimitSwitch = new DigitalInput(bottomLimitSwitchDioChannel);
        
        // Get encoder and PID controller
        encoder = primaryElevatorMotor.getEncoder();
        closedLoopController = primaryElevatorMotor.getClosedLoopController();

        // Configure the primary motor with PID
        SparkMaxConfig primaryConfig = new SparkMaxConfig();
        primaryConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MAX_CURRENT);
        
        primaryConfig.closedLoop
            .pid(kP, kI, kD)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .velocityFF(kFF)
            .outputRange(-1, 1);
        
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
        
        // Invert the secondary motor so they work together
        secondaryElevatorMotor.setInverted(true); // Using deprecated method but it's available
        
        // Reset encoder position
        resetEncoder();
        
        // Initialize simulation objects if in simulation mode
        if (RobotBase.isSimulation()) {
            initializeSimulation();
        }
        
        // Log initialization
        System.out.println("Elevator subsystem initialized");
    }
    
    private void initializeSimulation() {
        topLimitSwitchSim = new DIOSim(topLimitSwitch);
        bottomLimitSwitchSim = new DIOSim(bottomLimitSwitch);
        primaryElevatorMotorSim = new SparkMaxSim(primaryElevatorMotor, elevatorDCMotor);
        secondaryElevatorMotorSim = new SparkMaxSim(secondaryElevatorMotor, elevatorDCMotor);
    }

    /**
     * Move the elevator to a specific position
     * @param position Target position in encoder units
     */
    public void setPosition(double position) {
        // Safety check to ensure position is within bounds
        position = Math.min(Math.max(position, BOTTOM_THRESHOLD), TOP_THRESHOLD);
        
        // Store target position for soft start
        targetPosition = position;
        
        // Initialize soft start
        softStartPower = INITIAL_POWER * (position > getCurrentPosition() ? 1 : -1);
        isSoftStartActive = true;
        softStartTimer.reset();
        softStartTimer.start();
        
        System.out.println("Setting elevator position to " + position + " (current: " + getCurrentPosition() + ")");
        
        // The actual PID control will be applied in periodic() after soft start
    }

    /**
     * Move the elevator to a predefined level
     * @param level Level to move to (1-3)
     */
    public void moveToLevel(int level) {
        switch (level) {
            case 1:
                setPosition(LEVEL_1_HEIGHT);
                break;
            case 2:
                setPosition(LEVEL_2_HEIGHT);
                break;
            case 3:
                setPosition(LEVEL_3_HEIGHT);
                break;
            default:
                System.out.println("Invalid level: " + level);
                break;
        }
    }

    /**
     * Stop the elevator motors
     */
    public void stop() {
        System.out.println("Stopping elevator");
        primaryElevatorMotor.stopMotor();
        secondaryElevatorMotor.stopMotor();
        isSoftStartActive = false;
        softStartTimer.stop();
    }

    public double getCurrentPosition() {
        return encoder.getPosition();
    }
    
    public boolean isAtTarget() {
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

    @Override
    public void periodic() {
        // Safety checks - stop if either limit switch is triggered OR position exceeds thresholds
        if (isAtTop() || getCurrentPosition() > TOP_THRESHOLD) {
            if (primaryElevatorMotor.get() > 0) {
                stop();
                System.out.println("Elevator reached top limit");
            }
        } else if (isAtBottom() || getCurrentPosition() < BOTTOM_THRESHOLD) {
            if (primaryElevatorMotor.get() < 0) {
                stop();
                System.out.println("Elevator reached bottom limit");
            }
        }
        
        // Handle soft start logic
        if (isSoftStartActive) {
            double currentPosition = getCurrentPosition();
            
            // Calculate how far along the soft start we are (0.0 to 1.0)
            double softStartProgress = Math.min(softStartTimer.get() / RAMP_RATE, 1.0);
            
            if (softStartProgress < 1.0) {
                // Still in soft start phase
                // Gradually reduce soft start influence and increase PID influence
                // Manual PID calculation since we don't have calculate() method
                double proportional = kP * (targetPosition - currentPosition);
                double pidOutput = proportional; // Simple P controller
                
                double blendedOutput = (softStartPower * (1.0 - softStartProgress)) + 
                                      (pidOutput * softStartProgress);
                
                // Apply the blended output
                primaryElevatorMotor.set(blendedOutput);
                secondaryElevatorMotor.set(blendedOutput);
                System.out.println("Soft start: " + softStartProgress + ", Power: " + blendedOutput);
            } else {
                // Transition to full PID control
                isSoftStartActive = false;
                softStartTimer.stop();
                closedLoopController.setReference(targetPosition, ControlType.kPosition);
                System.out.println("Transitioning to full PID control");
            }
        }
        
        // Print periodic status every 50 calls (about once per second)
        if (periodicCounter++ % 50 == 0) {
            updateTelemetry();
        }
    }
    
    @Override
    public void simulationPeriodic() {
        // Update simulation models
        if (RobotBase.isSimulation()) {
            // Simulate elevator movement
            double motorVoltage = primaryElevatorMotor.get() * RobotController.getBatteryVoltage();
            double secondaryMotorVoltage = secondaryElevatorMotor.get() * RobotController.getBatteryVoltage();
            
            // Update motor simulations - use the simulation API methods
            // Note: For REV's simulation API, we need to check the actual methods available
            // This is a simplified approach that may need adjustment
            primaryElevatorMotorSim.iterate(motorVoltage, 0.0, 0.02);
            secondaryElevatorMotorSim.iterate(secondaryMotorVoltage, 0.0, 0.02);
            
            // Simulate limit switches based on position
            topLimitSwitchSim.setValue(getCurrentPosition() >= TOP_THRESHOLD);
            bottomLimitSwitchSim.setValue(getCurrentPosition() <= BOTTOM_THRESHOLD);
        }
    }
    
    private void updateTelemetry() {
        // Update SmartDashboard with current status
        SmartDashboard.putNumber("Elevator/Position", getCurrentPosition());
        SmartDashboard.putNumber("Elevator/Target", targetPosition);
        SmartDashboard.putBoolean("Elevator/AtTarget", isAtTarget());
        SmartDashboard.putBoolean("Elevator/TopLimit", isAtTop());
        SmartDashboard.putBoolean("Elevator/BottomLimit", isAtBottom());
        SmartDashboard.putNumber("Elevator/Primary/Current", primaryElevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/Primary/Voltage", primaryElevatorMotor.getBusVoltage());
        SmartDashboard.putNumber("Elevator/Primary/Speed", primaryElevatorMotor.get());
        SmartDashboard.putNumber("Elevator/Secondary/Current", secondaryElevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/Secondary/Voltage", secondaryElevatorMotor.getBusVoltage());
        SmartDashboard.putNumber("Elevator/Secondary/Speed", secondaryElevatorMotor.get());
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
    
    /**
     * For compatibility with existing code
     */
    public void goToLevel(int level) {
        moveToLevel(level);
    }
    
    /**
     * Check if the elevator is at the target position
     * @return true if at target position
     */
    public boolean atTargetPosition() {
        return isAtTarget();
    }
}
