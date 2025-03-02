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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;
    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    // Simulation
    private final DCMotor elevatorDCMotor;
    private final SparkMaxSim elevatorMotorSim;

    // Constants
    private static final double MAX_OUTPUT = 1.0;
    private static final double MIN_OUTPUT = -1.0;
    private static final double TOLERANCE = 0.5;

    // Elevator Position Constants (in encoder units)
    private static final double BOTTOM_THRESHOLD = 0.0;
    private static final double TOP_THRESHOLD = 100.0;  // Adjust based on actual max height
    private static final double LEVEL_1_HEIGHT = 10.0;  // Ground/Bottom level
    private static final double LEVEL_2_HEIGHT = 50.0;  // Mid level
    private static final double LEVEL_3_HEIGHT = 90.0;  // Top level

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

    public ElevatorSubsystem(int motorCanId, int topLimitSwitchId, int bottomLimitSwitchId) {
        // Initialize motor and controller
        elevatorMotor = new SparkMax(motorCanId, SparkMax.MotorType.kBrushless);
        encoder = elevatorMotor.getEncoder();
        closedLoopController = elevatorMotor.getClosedLoopController();

        // Configure motor and PID
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .velocityFF(kFF)
            .outputRange(MIN_OUTPUT, MAX_OUTPUT);
            
        elevatorMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        
        // Initialize limit switches
        topLimitSwitch = new DigitalInput(topLimitSwitchId);
        bottomLimitSwitch = new DigitalInput(bottomLimitSwitchId);

        encoder.setPosition(0);
        
        elevatorDCMotor = DCMotor.getNEO(1);
        elevatorMotorSim = new SparkMaxSim(elevatorMotor, elevatorDCMotor);
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
        if ((speed > 0 && isAtTop()) || (speed < 0 && isAtBottom())) {
            elevatorMotor.stopMotor();
            return;
        }

        // Apply speed limits
        speed = Math.max(MIN_OUTPUT, Math.min(MAX_OUTPUT, speed));
        elevatorMotor.set(speed);
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
        elevatorMotor.stopMotor();
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
        double currentPos = getCurrentPosition();
        if (Math.abs(currentPos - LEVEL_1_HEIGHT) < TOLERANCE) return 1;
        if (Math.abs(currentPos - LEVEL_2_HEIGHT) < TOLERANCE) return 2;
        if (Math.abs(currentPos - LEVEL_3_HEIGHT) < TOLERANCE) return 3;
        return 0;
    }

    @Override
    public void periodic() {
        // Add safety check
        if (isAtTop() && getCurrentPosition() > TOP_THRESHOLD) {
            stop();
            resetEncoder();  // Reset encoder to known position
        }
        if (isAtBottom() && getCurrentPosition() < BOTTOM_THRESHOLD) {
            stop();
            resetEncoder();  // Reset encoder to 0
        }
        
        updateTelemetry();
    }

    public void updateSimulatorState() {
        double positionError = targetPosition - encoder.getPosition();
        double velocityInchPerSec = positionError / 0.02;  // Basic simulation
        elevatorMotorSim.iterate(velocityInchPerSec, elevatorMotor.getBusVoltage(), 0.02);
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator/CurrentPosition", getCurrentPosition());
        SmartDashboard.putNumber("Elevator/TargetPosition", targetPosition);
        SmartDashboard.putNumber("Elevator/CurrentLevel", getCurrentLevel());
        SmartDashboard.putBoolean("Elevator/AtTop", isAtTop());
        SmartDashboard.putBoolean("Elevator/AtBottom", isAtBottom());
        SmartDashboard.putBoolean("Elevator/AtTarget", atTargetPosition());
        SmartDashboard.putNumber("Elevator/Current", elevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/Voltage", elevatorMotor.getBusVoltage());
    }
}