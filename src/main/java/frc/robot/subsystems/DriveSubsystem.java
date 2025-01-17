package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // Locations for the swerve drive modules relative to the robot center
    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    // Motor controllers for the swerve drive modules
    private final SwerveModule m_frontLeft = new SwerveModule(1, 2, false, false);
    private final SwerveModule m_frontRight = new SwerveModule(3, 4, true, true);
    private final SwerveModule m_backLeft = new SwerveModule(5, 6, false, false);
    private final SwerveModule m_backRight = new SwerveModule(7, 8, true, true);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private int updateCounter = 0;

    public DriveSubsystem() {
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     */
    public void drive(double xSpeed, double ySpeed, double rot) {
        // Debug input values
        SmartDashboard.putNumber("Drive/Input/X", xSpeed);
        SmartDashboard.putNumber("Drive/Input/Y", ySpeed);
        SmartDashboard.putNumber("Drive/Input/Rot", rot);

        // If all inputs are zero, stop the motors
        if (Math.abs(xSpeed) < 1E-6 && Math.abs(ySpeed) < 1E-6 && Math.abs(rot) < 1E-6) {
            stop();
            return;
        }

        // Convert to meters per second
        xSpeed = xSpeed * 4.0; // Maximum speed of 4 meters per second
        ySpeed = ySpeed * 4.0;
        rot = rot * 4.0;

        var swerveModuleStates = kinematics.toSwerveModuleStates(
            new ChassisSpeeds(xSpeed, ySpeed, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 4.0);

        // Debug output values
        SmartDashboard.putNumber("Drive/FL/Speed", swerveModuleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("Drive/FL/Angle", swerveModuleStates[0].angle.getDegrees());

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    @Override
    public void periodic() {
        // Only update SmartDashboard every 10 cycles to reduce NT traffic
        updateCounter++;
        if (updateCounter >= 10) {
            try {
                // Front Left Module
                SmartDashboard.putNumber("Swerve/Front Left/Drive Speed", m_frontLeft.getDriveSpeed());
                SmartDashboard.putNumber("Swerve/Front Left/Steer Angle", m_frontLeft.getSteerAngle());
                
                // Front Right Module
                SmartDashboard.putNumber("Swerve/Front Right/Drive Speed", m_frontRight.getDriveSpeed());
                SmartDashboard.putNumber("Swerve/Front Right/Steer Angle", m_frontRight.getSteerAngle());
                
                // Back Left Module
                SmartDashboard.putNumber("Swerve/Back Left/Drive Speed", m_backLeft.getDriveSpeed());
                SmartDashboard.putNumber("Swerve/Back Left/Steer Angle", m_backLeft.getSteerAngle());
                
                // Back Right Module
                SmartDashboard.putNumber("Swerve/Back Right/Drive Speed", m_backRight.getDriveSpeed());
                SmartDashboard.putNumber("Swerve/Back Right/Steer Angle", m_backRight.getSteerAngle());
            } catch (Exception e) {
                System.err.println("Error updating SmartDashboard: " + e.getMessage());
            }
            updateCounter = 0;
        }
    }
}
