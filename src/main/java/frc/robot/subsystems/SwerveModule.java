package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;
    
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPIDController;

    private final boolean driveEncoderReversed;
    private final boolean turningEncoderReversed;

    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed) {
        this.driveEncoderReversed = driveEncoderReversed;
        this.turningEncoderReversed = turningEncoderReversed;

        driveMotor = new SparkMax(driveMotorChannel, SparkMax.MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorChannel, SparkMax.MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        // PID controller for turning, assumes the encoder reports degrees and handles wrapping
        turningPIDController = new PIDController(0.5, 0, 0);
        turningPIDController.enableContinuousInput(-180, 180);

        // Configure encoders and motors
        restoreFactoryDefaults(driveMotor);
        restoreFactoryDefaults(turningMotor);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            new Rotation2d(Math.toRadians(getTurningPosition()))
        );
    }

    public void setDesiredState(SwerveModuleState state) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, 
            new Rotation2d(Math.toRadians(getTurningPosition())));

        // Calculate the drive output from the drive encoder velocity
        final double driveOutput = optimizedState.speedMetersPerSecond;
        
        // Calculate the turning motor output from the turning PID controller
        final double turnOutput = turningPIDController.calculate(
            getTurningPosition(), optimizedState.angle.getDegrees());

        driveMotor.set(driveOutput);
        turningMotor.set(turnOutput);
    }

    private double getDriveVelocity() {
        double velocity = driveEncoder.getVelocity();
        return driveEncoderReversed ? -velocity : velocity;
    }

    private double getTurningPosition() {
        double position = turningEncoder.getPosition();
        return turningEncoderReversed ? -position : position;
    }

    public double getDriveSpeed() {
        return driveMotor.get();
    }

    public double getSteerAngle() {
        return getTurningPosition();
    }

    public void stop() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }

    /**
     * Returns the current position of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePosition(),
            new Rotation2d(Math.toRadians(getTurningPosition()))
        );
    }

    /**
     * Returns the current position of the drive encoder in meters
     */
    public double getDrivePosition() {
        double position = driveEncoder.getPosition();
        return position * (driveEncoderReversed ? -1.0 : 1.0);
    }

    private void restoreFactoryDefaults(SparkMax motor) {
        SparkMaxConfig config = new SparkMaxConfig();

        //config
            //.inverted(true)
            //.idleMode(IdleMode.kBrake)
            //.smartCurrentConfig(new SmartCurrentConfig().stallLimit(40).freeLimit(40))
            //.openLoopRampRate(0.5)
            //.closedLoopRampRate(0.5)
            //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //.pid(1.0, 0.0, 0.0);
            
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
