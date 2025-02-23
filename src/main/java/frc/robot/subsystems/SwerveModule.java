package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;
    
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkClosedLoopController driveClosedLoopController;
    private final SparkClosedLoopController turningClosedLoopController;

    // robot chasis is not angled perfectly with each module
    private double chasisAngularOffset;

    // doubles to mark the wanted driveSpeed, and turningPosition
    private double desiredSpeed;
    private double desiredAngle;

    // motor and simulated versions of the drive and turning motors
    private final DCMotor driveDCMotor;
    private final DCMotor turningDCMotor; 
    private final SparkMaxSim driveMotorSim;
    private final SparkMaxSim turningMotorSim;

    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            double angularOffset,
            boolean inverted) {

        driveMotor = new SparkMax(driveMotorChannel, SparkMax.MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorChannel, SparkMax.MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder();

        driveClosedLoopController = driveMotor.getClosedLoopController();
        turningClosedLoopController = turningMotor.getClosedLoopController();
        
        // Configure encoders and motors
        driveMotor.configure(inverted ? Configs.SwerveModule.drivingInvertedConfig : Configs.SwerveModule.drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(Configs.SwerveModule.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        chasisAngularOffset = angularOffset;
        driveEncoder.setPosition(0);
        
        // setup simulated motors
        driveDCMotor = DCMotor.getNEO(1);
        turningDCMotor = DCMotor.getNEO(1);

        driveMotorSim = new SparkMaxSim(driveMotor, driveDCMotor);
        turningMotorSim = new SparkMaxSim(turningMotor, turningDCMotor);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            new Rotation2d(getTurningPosition())
        );
    }
    /**
     * Returns the current position of the module
     */
    public SwerveModulePosition getPosition() {
        // apply angular offset to encoder position to get position relative to chasis
        return new SwerveModulePosition(
            getDrivePosition(),
            new Rotation2d(getTurningPosition())
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // apply chasis angular offset to the desired state
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chasisAngularOffset));
        
        // Optimize the reference state to avoid spinning further than 90 degrees
        correctedDesiredState.optimize(new Rotation2d(turningEncoder.getPosition()));

        // Calculate the drive output from the drive encoder velocity
        desiredSpeed = correctedDesiredState.speedMetersPerSecond;
        desiredAngle = correctedDesiredState.angle.getRadians();

        // PID Controllers sets the velocity and angle pos as a reference to KEEP A CONSISTENT VALUE
        driveClosedLoopController.setReference(desiredSpeed, ControlType.kVelocity);
        turningClosedLoopController.setReference(desiredAngle, ControlType.kPosition);
    }

    private double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }
    // gives the robot relative turning position (gives 0 degrees if robot moving 0 degrees)
    private double getTurningPosition() {
        return turningEncoder.getPosition() - chasisAngularOffset;
    }

    public double getDriveSpeed() {
        return getDriveVelocity();
    }

    public double getSteerAngle() {
        return getTurningPosition();
    }

    public void stop() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();

        desiredSpeed = 0;
        desiredAngle = 0;
    }

    /**
     * Returns the current position of the drive encoder in meters
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    // updates the states of the simulated motors (velocity, and pos), which automatically updates the encoders of the actual motors
    public void updateSimulatorState() {
        driveMotorSim.iterate(desiredSpeed, driveMotor.getBusVoltage(), 0.02);
        
        double positionError = desiredAngle - turningEncoder.getPosition();
        double velocityRadPerSec = positionError / 0.02;

        turningMotorSim.iterate(velocityRadPerSec, turningMotor.getBusVoltage(), 0.02);
    } 
}
