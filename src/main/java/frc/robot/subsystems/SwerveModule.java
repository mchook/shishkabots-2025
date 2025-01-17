package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;
    
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

        driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMax.MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorChannel, CANSparkMax.MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        // PID controller for turning, assumes the encoder reports degrees and handles wrapping
        turningPIDController = new PIDController(0.5, 0, 0);
        turningPIDController.enableContinuousInput(-180, 180);

        // Configure encoders and motors
        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();
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
}
