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
    private static final double BOTTOM_THRESHOLD = 5.0;
    private static final double TOP_THRESHOLD = 40.0;  // Adjust based on actual max height
    private static final double LEVEL_1_HEIGHT = 10.0;  // Ground/Bottom level
    private static final double LEVEL_2_HEIGHT = 20.0;  // Mid level
    private static final double LEVEL_3_HEIGHT = 30.0;  // Top level

    // PID Constants - Tune these values during testing
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kFF = 0.0;
    
    // Position Control
    private double targetPosition = 0.0;

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
        
        // Get encoder and controller from primary motor
        encoder = primaryElevatorMotor.getEncoder();
        closedLoopController = primaryElevatorMotor.getClosedLoopController();

        // Configure the primary motor with PID
        SparkMaxConfig primaryConfig = new SparkMaxConfig();
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
        
        // Initialize the secondary motor
        secondaryElevatorMotor.set(0);
        
        // Initialize limit switches
        topLimitSwitch = new DigitalInput(topLimitSwitchId);
        bottomLimitSwitch = new DigitalInput(bottomLimitSwitchId);

        encoder.setPosition(0);
        
        // Initialize simulation
        elevatorDCMotor = DCMotor.getNEO(2); // Using 2 NEOs
        primaryElevatorMotorSim = new SparkMaxSim(primaryElevatorMotor, elevatorDCMotor);
        secondaryElevatorMotorSim = new SparkMaxSim(secondaryElevatorMotor, elevatorDCMotor);
    }

    public void setTargetPosition(double position) {
        // Clamp position within safe bounds
        position = Math.max(BOTTOM_THRESHOLD, Math.min(position, TOP_THRESHOLD));
        targetPosition = position;
        closedLoopController.setReference(position, ControlType.kPosition);
    }

    public double getCurrentPosition() {
        return encoder.getPosition();
    }

    public void setSpeed(double speed) {
        // Apply speed limits
        speed = Math.max(MIN_OUTPUT, Math.min(MAX_OUTPUT, speed));
        primaryElevatorMotor.set(speed);
        secondaryElevatorMotor.set(speed);  // Keep motors synchronized
    }

    public boolean isAtTop() {
        return !topLimitSwitch.get();
    }

    public boolean isAtBottom() {
        return !bottomLimitSwitch.get();
    }

    public boolean atTargetPosition() {
        return Math.abs(getCurrentPosition() - targetPosition) < TOLERANCE;
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public void stop() {
        primaryElevatorMotor.stopMotor();
        secondaryElevatorMotor.stopMotor();  // Stop both motors
    }

    /**
     * Sets the elevator to a predefined level
     * @param level 1 for bottom, 2 for middle, 3 for top
     */
    public void goToLevel(int level) {
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
        // Manually synchronize the secondary motor with the primary
        secondaryElevatorMotor.set(primaryElevatorMotor.get());
        
        // Safety checks - stop if either limit switch is triggered OR position exceeds thresholds
        if (isAtTop() || getCurrentPosition() > TOP_THRESHOLD) {
            stop();
        }
        
        if (isAtBottom() || getCurrentPosition() < BOTTOM_THRESHOLD) {
            stop();
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
        SmartDashboard.putNumber("Elevator/Current", primaryElevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/Voltage", primaryElevatorMotor.getBusVoltage());
    }
}