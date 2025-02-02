package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class DriveSubsystem extends SubsystemBase {
    // Locations for the swerve drive modules relative to the robot center
    private final Translation2d m_frontLeftLocation = Constants.FRONT_LEFT_LOCATION;
    private final Translation2d m_frontRightLocation = Constants.FRONT_RIGHT_LOCATION;
    private final Translation2d m_backLeftLocation = Constants.BACK_LEFT_LOCATION;
    private final Translation2d m_backRightLocation = Constants.BACK_RIGHT_LOCATION;

    // Motor controllers for the swerve drive modules
    private final SwerveModule m_frontLeft = new SwerveModule(1, 2, false, false);
    private final SwerveModule m_frontRight = new SwerveModule(3, 4, true, true);
    private final SwerveModule m_backLeft = new SwerveModule(5, 6, false, false);
    private final SwerveModule m_backRight = new SwerveModule(7, 8, true, true);

    
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    // Slew rate limiters to make joystick inputs more gentle
    private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(Constants.MAX_MAGNITUDE_SLEW_RATE);
    private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(Constants.MAX_MAGNITUDE_SLEW_RATE);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.MAX_ROTATIONAL_SLEW_RATE_RPS);

    private final Pigeon2 m_gyro = new Pigeon2(Constants.PIGEON_CAN_ID); // Update the ID based on your Pigeon's CAN ID

    // initialize the field for simulator tracking
    private final Field2d m_field = new Field2d();

    // Odometry for tracking robot pose
    private final SwerveDriveOdometry m_odometry;

    private int updateCounter = 0;

    private DoubleLogEntry m_speedLog;
    private DoubleLogEntry m_headingLog;

    public DriveSubsystem() {
        // Reset the gyro
        m_gyro.reset();

        // log field into smartdashboard
        SmartDashboard.putData("Field", m_field);

        // Initialize odometry
        m_odometry = new SwerveDriveOdometry(
            kinematics,
            m_gyro.getRotation2d(),  // example used built-in method to return as rotation2d unit
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }, new Pose2d(8.2, 5, new Rotation2d())
        );

        // Initialize DataLogManager entries
        DataLog log = DataLogManager.getLog();
        m_speedLog = new DoubleLogEntry(log, "/drive/speed");
        m_headingLog = new DoubleLogEntry(log, "/drive/heading");
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

        // Convert the commanded speeds from [-1, 1] to real speeds
        xSpeed = xSpeed * Constants.MAX_SPEED_IN_MPS;
        ySpeed = ySpeed * Constants.MAX_SPEED_IN_MPS;
        rot = rot * Constants.MAX_ANGULAR_SPEED_IN_RPS;

        // Apply slew rate limiters to smooth out the inputs
        xSpeed = m_xSpeedLimiter.calculate(xSpeed);
        ySpeed = m_ySpeedLimiter.calculate(ySpeed);
        rot = m_rotLimiter.calculate(rot);

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

    /**
     * Returns the gyro rotation as a Rotation2d object
     */
    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
    }

    /**
     * Returns the current pose of the robot
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to a known pose
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            getGyroRotation(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            },
            pose
        );
    }

    @Override
    public void periodic() {
        // Update odometry
        m_odometry.update(
            getGyroRotation(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
        );
        // set robot position in the field
        m_field.setRobotPose(m_odometry.getPoseMeters());
        // Only update SmartDashboard every 10 cycles to reduce NT traffic
        updateCounter++;
        if (updateCounter >= 10) {
            try {

                // log array of all swerve modules to be put into advantagescope simulation
            double loggingState[] = {
                m_frontLeft.getSteerAngle(),
                m_frontLeft.getDriveSpeed(),
                m_frontRight.getSteerAngle(),
                m_frontRight.getDriveSpeed(),
                m_backLeft.getSteerAngle(),
                m_backLeft.getDriveSpeed(),
                m_backRight.getSteerAngle(),
                m_backRight.getDriveSpeed()
            };

            SmartDashboard.putNumberArray("SwerveModuleStates", loggingState);
                /* // Front Left Module
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
                SmartDashboard.putNumber("Swerve/Back Right/Steer Angle", m_backRight.getSteerAngle()); */

                // Add odometry data to SmartDashboard
                var pose = getPose();
                SmartDashboard.putNumber("Pose/X", pose.getX());
                SmartDashboard.putNumber("Pose/Y", pose.getY());
                SmartDashboard.putNumber("Pose/Rotation", pose.getRotation().getDegrees());
                SmartDashboard.putNumber("Gyro/Angle", getGyroRotation().getDegrees());

                // Log important values
                var chassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
                double speed = Math.sqrt(
                    chassisSpeeds.vxMetersPerSecond * chassisSpeeds.vxMetersPerSecond +
                    chassisSpeeds.vyMetersPerSecond * chassisSpeeds.vyMetersPerSecond
                );

                // Log to DataLog (saved to file)
                m_speedLog.append(speed);
                m_headingLog.append(getGyroRotation().getDegrees());

                // Log to SmartDashboard (network tables, viewable in Shuffleboard)
                SmartDashboard.putNumber("Drive/Speed (m/s)", speed);
                SmartDashboard.putNumber("Drive/Heading (deg)", getGyroRotation().getDegrees());
                SmartDashboard.putString("Drive/Pose", getPose().toString());
            } catch (Exception e) {
                System.err.println("Error updating SmartDashboard: " + e.getMessage());
            }
            updateCounter = 0;
        }
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
    }

    private Rotation2d getHeading() {
        return getGyroRotation();
    }
}
