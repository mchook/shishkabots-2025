package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final SparkClosedLoopController closedLoopController;
    
    // Color sensor for game piece detection
    private ColorSensorV3 colorSensor;
    private static final int PROXIMITY_THRESHOLD = 100; // Adjust based on testing
    private int lastProximity = 0;
    private boolean hasColorSensor = false;

    // Motor configuration constants
    private static final double SHOOT_VELOCITY = 3000.0; // RPM
    private static final int MAX_CURRENT = 40; // Amps
    private static final double kP = 0.005;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kFF = 0.000175;

    public ShooterSubsystem(int leftMotorCanId, int rightMotorCanId) {
        leftMotor = new SparkMax(leftMotorCanId, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorCanId, MotorType.kBrushless);
        closedLoopController = leftMotor.getClosedLoopController();

        // Try to initialize color sensor on the I2C port
        try {
            colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
            hasColorSensor = true;
            System.out.println("Color sensor initialized successfully");
        } catch (Exception e) {
            System.out.println("Color sensor not detected, running without game piece detection");
            hasColorSensor = false;
        }

        // Configure the left motor (leader)
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig
            .idleMode(IdleMode.kCoast)  // Coast mode for less wear on the motors
            .smartCurrentLimit(MAX_CURRENT);
        
        leftConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .velocityFF(kFF)
            .outputRange(-1, 1);
            
        leftMotor.configure(
            leftConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // Configure the right motor (follower)
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(MAX_CURRENT);
            
        rightMotor.configure(
            rightConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // Initialize motors stopped
        stop();
        
        System.out.println("Shooter subsystem initialized");
    }

    public void shoot() {
        System.out.println("Setting shooter velocity to " + SHOOT_VELOCITY + " RPM");
        closedLoopController.setReference(SHOOT_VELOCITY, ControlType.kVelocity);
    }

    public void stop() {
        System.out.println("Stopping shooter motors");
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    /**
     * Checks if a game piece has entered the shooter using the color sensor's proximity reading
     * @return true if a game piece is detected at the entry of the shooter, false if no sensor
     */
    public boolean hasGamePieceEntered() {
        if (!hasColorSensor) {
            return false;
        }

        int proximity = colorSensor.getProximity();
        boolean isClose = proximity > PROXIMITY_THRESHOLD;
        
        // If we detect a sudden increase in proximity, a game piece likely entered
        if (isClose && lastProximity <= PROXIMITY_THRESHOLD) {
            var detectedColor = colorSensor.getColor();
            System.out.println(String.format("Game piece detected! Color: R=%.2f, G=%.2f, B=%.2f, Proximity=%d",
                detectedColor.red, detectedColor.green, detectedColor.blue, proximity));
        }
        
        lastProximity = proximity;
        return isClose;
    }

    /**
     * Checks if a game piece has exited the shooter by checking if proximity drops after being high
     * @return true if a game piece is detected leaving the shooter, false if no sensor
     */
    public boolean hasGamePieceExited() {
        if (!hasColorSensor) {
            return false;
        }

        int proximity = colorSensor.getProximity();
        boolean hasExited = lastProximity > PROXIMITY_THRESHOLD && proximity <= PROXIMITY_THRESHOLD;
        
        if (hasExited) {
            System.out.println("Game piece has exited the shooter");
        }
        
        lastProximity = proximity;
        return hasExited;
    }

    @Override
    public void periodic() {
        // Right motor runs at opposite speed of left motor
        rightMotor.set(-leftMotor.get());
        
        // Update telemetry
        updateTelemetry();

        // Log game piece status changes for debugging (only if we have a sensor)
        if (hasColorSensor) {
            if (hasGamePieceEntered()) {
                System.out.println("Game piece detected entering shooter");
            }
            if (hasGamePieceExited()) {
                System.out.println("Game piece detected exiting shooter");
            }
        }
    }

    private void updateTelemetry() {
        // Left motor telemetry
        SmartDashboard.putNumber("Shooter/Left/Velocity", leftMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Left/Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Left/Voltage", leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Shooter/Left/Speed", leftMotor.get());
        
        // Right motor telemetry
        SmartDashboard.putNumber("Shooter/Right/Velocity", rightMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Right/Current", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Right/Voltage", rightMotor.getBusVoltage());
        SmartDashboard.putNumber("Shooter/Right/Speed", rightMotor.get());

        // Color sensor telemetry (only if sensor is present)
        if (hasColorSensor) {
            var color = colorSensor.getColor();
            int proximity = colorSensor.getProximity();
            SmartDashboard.putNumber("Shooter/Sensor/Red", color.red);
            SmartDashboard.putNumber("Shooter/Sensor/Green", color.green);
            SmartDashboard.putNumber("Shooter/Sensor/Blue", color.blue);
            SmartDashboard.putNumber("Shooter/Sensor/Proximity", proximity);
        }
        SmartDashboard.putBoolean("Shooter/GamePiecePresent", hasGamePieceEntered());
        SmartDashboard.putBoolean("Shooter/GamePieceExited", hasGamePieceExited());
    }
}
