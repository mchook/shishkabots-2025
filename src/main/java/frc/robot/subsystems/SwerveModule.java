package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;
    
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final SparkClosedLoopController drivePIDController;
    private final SparkClosedLoopController turningPIDController;

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

        drivePIDController = driveMotor.getClosedLoopController();
        turningPIDController = turningMotor.getClosedLoopController();

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        // Configure encoders and motors
        setSparkMaxConfig(driveMotor, driveEncoderReversed);
        setSparkMaxConfig(turningMotor, turningEncoderReversed);

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

        // PID Controllers sets the velocity and angle pos as a reference to KEEP A CONSISTENT VALUE
        drivePIDController.setReference(driveOutput, ControlType.kVelocity);
        turningPIDController.setReference(optimizedState.angle.getRadians() * Constants.RadiansToMeters, ControlType.kPosition);

        // motor control already handled by PID Controller. Set commands are kept for testing for now.
        driveMotor.set(driveOutput);
        turningMotor.set(optimizedState.angle.getDegrees());
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
        // return getTurningPosition();
        return turningMotor.get();
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

    private void setSparkMaxConfig(SparkMax motor, boolean isInverted) {
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .inverted(isInverted)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(Constants.ROTATIONS_TO_METERS)
            .velocityConversionFactor(Constants.RPMToMetersPerSec);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.0, 0.0, 0.0);
            
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
