package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
    // Locations for the swerve drive modules relative to the robot center
    private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

    // Motor controllers for the swerve drive modules
    private final CANSparkMax frontLeftDriveMotor = new CANSparkMax(4, MotorType.kBrushless);
    private final CANSparkMax frontLeftSteerMotor = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax frontRightDriveMotor = new CANSparkMax(6, MotorType.kBrushless);
    private final CANSparkMax frontRightSteerMotor = new CANSparkMax(5, MotorType.kBrushless);
    private final CANSparkMax backLeftDriveMotor = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax backLeftSteerMotor = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax backRightDriveMotor = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax backRightSteerMotor = new CANSparkMax(7, MotorType.kBrushless);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private int updateCounter = 0;

    public DriveSubsystem() {
        // Invert the right side motors
        frontRightDriveMotor.setInverted(true);
        frontRightSteerMotor.setInverted(true);
        backRightDriveMotor.setInverted(true);
        backRightSteerMotor.setInverted(true);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     */
    public void drive(double xSpeed, double ySpeed, double rot) {
        // Convert to meters per second
        xSpeed = xSpeed * 4.0; // Maximum speed of 4 meters per second
        ySpeed = ySpeed * 4.0;
        rot = rot * 4.0;

        var swerveModuleStates = kinematics.toSwerveModuleStates(
            new ChassisSpeeds(xSpeed, ySpeed, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 4.0);

        frontLeftDriveMotor.set(swerveModuleStates[0].speedMetersPerSecond);
        frontLeftSteerMotor.set(swerveModuleStates[0].angle.getRadians());
        frontRightDriveMotor.set(swerveModuleStates[1].speedMetersPerSecond);
        frontRightSteerMotor.set(swerveModuleStates[1].angle.getRadians());
        backLeftDriveMotor.set(swerveModuleStates[2].speedMetersPerSecond);
        backLeftSteerMotor.set(swerveModuleStates[2].angle.getRadians());
        backRightDriveMotor.set(swerveModuleStates[3].speedMetersPerSecond);
        backRightSteerMotor.set(swerveModuleStates[3].angle.getRadians());
    }

    public void stop() {
        frontLeftDriveMotor.stopMotor();
        frontLeftSteerMotor.stopMotor();
        frontRightDriveMotor.stopMotor();
        frontRightSteerMotor.stopMotor();
        backLeftDriveMotor.stopMotor();
        backLeftSteerMotor.stopMotor();
        backRightDriveMotor.stopMotor();
        backRightSteerMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // Only update SmartDashboard every 10 cycles to reduce NT traffic
        updateCounter++;
        if (updateCounter >= 10) {
            try {
                // Front Left Module
                SmartDashboard.putNumber("Swerve/Front Left/Drive Speed", frontLeftDriveMotor.get());
                SmartDashboard.putNumber("Swerve/Front Left/Steer Angle", frontLeftSteerMotor.get());
                
                // Front Right Module
                SmartDashboard.putNumber("Swerve/Front Right/Drive Speed", frontRightDriveMotor.get());
                SmartDashboard.putNumber("Swerve/Front Right/Steer Angle", frontRightSteerMotor.get());
                
                // Back Left Module
                SmartDashboard.putNumber("Swerve/Back Left/Drive Speed", backLeftDriveMotor.get());
                SmartDashboard.putNumber("Swerve/Back Left/Steer Angle", backLeftSteerMotor.get());
                
                // Back Right Module
                SmartDashboard.putNumber("Swerve/Back Right/Drive Speed", backRightDriveMotor.get());
                SmartDashboard.putNumber("Swerve/Back Right/Steer Angle", backRightSteerMotor.get());
            } catch (Exception e) {
                System.err.println("Error updating SmartDashboard: " + e.getMessage());
            }
            updateCounter = 0;
        }
    }
}
