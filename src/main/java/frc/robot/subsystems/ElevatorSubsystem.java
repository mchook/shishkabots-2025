package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;

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
    
    // Position Control
    private double targetPosition = 0.0;

    public ElevatorSubsystem(int motorCanId, int topLimitSwitchId, int bottomLimitSwitchId) {
        // Initialize motor and controller
        elevatorMotor = new SparkMax(motorCanId, SparkMax.MotorType.kBrushless);
        encoder = elevatorMotor.getEncoder();
        closedLoopController = elevatorMotor.getClosedLoopController();

        // Configure motor
        elevatorMotor.configure(
            Configs.Elevator.motorConfig,
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

    @Override
    public void periodic() {
        updateTelemetry();
    }

    public void updateSimulatorState() {
        double positionError = targetPosition - encoder.getPosition();
        double velocityInchPerSec = positionError / 0.02;  // Basic simulation
        elevatorMotorSim.iterate(velocityInchPerSec, elevatorMotor.getBusVoltage(), 0.02);
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator Position", getCurrentPosition());
        SmartDashboard.putNumber("Elevator Target", targetPosition);
        SmartDashboard.putBoolean("Elevator At Top", isAtTop());
        SmartDashboard.putBoolean("Elevator At Bottom", isAtBottom());
        SmartDashboard.putNumber("Elevator Current", elevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Voltage", elevatorMotor.getBusVoltage());
        SmartDashboard.putBoolean("Elevator At Target", atTargetPosition());
    }
}