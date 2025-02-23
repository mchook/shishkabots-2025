package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class DriveSubsystem extends SubsystemBase {
    // Locations for the swerve drive modules relative to the robot center
    private final Translation2d m_frontLeftLocation = DriveConstants.FRONT_LEFT_LOCATION;
    private final Translation2d m_frontRightLocation = DriveConstants.FRONT_RIGHT_LOCATION;
    private final Translation2d m_backLeftLocation = DriveConstants.BACK_LEFT_LOCATION;
    private final Translation2d m_backRightLocation = DriveConstants.BACK_RIGHT_LOCATION;

    // Motor controllers for the swerve drive modules
    private final SwerveModule m_frontLeft = new SwerveModule(
        DriveConstants.DRIVE_FRONT_LEFT_CAN_ID, 
        DriveConstants.DRIVE_TURN_FRONT_LEFT_CAN_ID, 
        DriveConstants.FRONT_LEFT_CHASIS_ANGULAR_OFFSET,
        true);

    private final SwerveModule m_frontRight = new SwerveModule(
        DriveConstants.DRIVE_FRONT_RIGHT_CAN_ID, 
        DriveConstants.DRIVE_TURN_FRONT_RIGHT_CAN_ID, 
        DriveConstants.FRONT_RIGHT_CHASIS_ANGULAR_OFFSET,
        false);
        
    private final SwerveModule m_backLeft = new SwerveModule(
        DriveConstants.DRIVE_REAR_LEFT_CAN_ID, 
        DriveConstants.DRIVE_TURN_REAR_LEFT_CAN_ID, 
        DriveConstants.BACK_LEFT_CHASIS_ANGULAR_OFFSET,
        true);

    private final SwerveModule m_backRight = new SwerveModule(
        DriveConstants.DRIVE_REAR_RIGHT_CAN_ID, 
        DriveConstants.DRIVE_TURN_REAR_RIGHT_CAN_ID, 
        DriveConstants.BACK_RIGHT_CHASIS_ANGULAR_OFFSET,
        false);

    
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    // Slew rate limiters to make joystick inputs more gentle
    private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(DriveConstants.MAX_MAGNITUDE_SLEW_RATE);
    private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(DriveConstants.MAX_MAGNITUDE_SLEW_RATE);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.MAX_ROTATIONAL_SLEW_RATE_RPS);

    private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.PIGEON_CAN_ID); // Update the ID based on your Pigeon's CAN ID
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
            getGyroRotation(),  // example used built-in method to return as rotation2d unit
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }, new Pose2d(8.2, 5, new Rotation2d())
        );

        try{
            DriveConstants.pathPlannerConfig = RobotConfig.fromGUISettings();
          } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
          }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
                ),
                DriveConstants.pathPlannerConfig, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false; 
                }, 
                this // Reference to this subsystem to set requirements
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
        xSpeed = xSpeed * DriveConstants.MAX_SPEED_IN_MPS;
        ySpeed = ySpeed * DriveConstants.MAX_SPEED_IN_MPS;
        rot = rot * DriveConstants.MAX_ANGULAR_SPEED_IN_RPS;

        // Apply slew rate limiters to smooth out the inputs
        //xSpeed = m_xSpeedLimiter.calculate(xSpeed);
        //ySpeed = m_ySpeedLimiter.calculate(ySpeed);
        //rot = m_rotLimiter.calculate(rot);

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        var swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 4.0);

        SmartDashboard.putNumber("Chasis Speeds X", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Chasis Speeds Y", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Chasis Speeds Rotation", speeds.omegaRadiansPerSecond);

        // Debug output values
        SmartDashboard.putNumber("Drive/FL/Speed", swerveModuleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("Drive/FL/Angle", swerveModuleStates[0].angle.getDegrees());

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void drive(ChassisSpeeds speeds) {
        double xSpeed = speeds.vxMetersPerSecond;
        double ySpeed = speeds.vyMetersPerSecond;
        double rot = speeds.omegaRadiansPerSecond;
        drive(xSpeed, ySpeed, rot);
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

    public ChassisSpeeds getCurrentSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
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
    
    @Override
    public void simulationPeriodic() {
        m_frontLeft.updateSimulatorState();
        m_frontRight.updateSimulatorState();
        m_backLeft.updateSimulatorState();
        m_backRight.updateSimulatorState();

        double angularVelocity = kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond;
        updateGyroSimulatorState(angularVelocity);
    }

    public void updateGyroSimulatorState(double angularVelocity) {
        // convert radians per second to degrees per second
        double angularVelocityDegrees = angularVelocity * (180 / Math.PI);
        double newYaw = m_gyro.getYaw().getValueAsDouble() +  angularVelocityDegrees * 0.02;
        m_gyro.getSimState().setRawYaw(newYaw);
    }
}
