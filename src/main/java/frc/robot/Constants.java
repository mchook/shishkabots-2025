package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;

public class Constants {
      public static final class LimelightConstants {
    // Translation from the center of the robot to the camera (robot coordinates).
    public static final Translation3d kCameraToRobot =
        new Translation3d(0, 0.3319, 0.465); // TODO: Not final
    public static final double kResolutionWidth = 1280;
    public static final double kResolutionHeight = 960;
    // Limelight publish to `updateData()` delay
    public static final double delayMillis = 10; // TODO: Make more accurate
  }
    public static final class Locations {
        public static final Translation2d[] leftBranchLocations = {
            new Translation2d(5.83809, 3.85728)
        };
    }
    public static final class DriveConstants {
        public static RobotConfig pathPlannerConfig;
        // Chassis configuration
        public static final double TRACK_WIDTH = Units.Inches.of(26.5).in(Units.Meters);
        // Distance between centers of right and left wheels on robot
        public static final double WHEEL_BASE = Units.Inches.of(26.5).in(Units.Meters);
        // Locations for the swerve drive modules relative to the robot center
        public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

        // angular offsets of modules relative to the chasis
        /* public static final double FRONT_LEFT_CHASIS_ANGULAR_OFFSET = -Math.PI/2;
        public static final double FRONT_RIGHT_CHASIS_ANGULAR_OFFSET = 0;
        public static final double BACK_LEFT_CHASIS_ANGULAR_OFFSET = Math.PI;
        public static final double BACK_RIGHT_CHASIS_ANGULAR_OFFSET = Math.PI/2; */
        public static final double FRONT_LEFT_CHASIS_ANGULAR_OFFSET = 0;
        public static final double FRONT_RIGHT_CHASIS_ANGULAR_OFFSET = 0;
        public static final double BACK_LEFT_CHASIS_ANGULAR_OFFSET = 0;
        public static final double BACK_RIGHT_CHASIS_ANGULAR_OFFSET = 0;

        // Spark Max CAN Ids
        public static final int DRIVE_FRONT_LEFT_CAN_ID = 1;
        public static final int DRIVE_REAR_LEFT_CAN_ID = 5;
        public static final int DRIVE_FRONT_RIGHT_CAN_ID = 3;
        public static final int DRIVE_REAR_RIGHT_CAN_ID = 7;

        public static final int DRIVE_TURN_FRONT_LEFT_CAN_ID = 2;
        public static final int DRIVE_TURN_REAR_LEFT_CAN_ID = 6;
        public static final int DRIVE_TURN_FRONT_RIGHT_CAN_ID = 4;
        public static final int DRIVE_TURN_REAR_RIGHT_CAN_ID = 8;
        
        public static final int PIGEON_CAN_ID = 8;

        public static final double MAX_SPEED_IN_MPS = 4.0; // meters per second
        public static final double MAX_ANGULAR_SPEED_IN_RPS = 1 * Math.PI; // radians per second

        public static final double MAX_DIRECTION_SLEW_RATE_RPS = 1.3; // radians per second
        public static final double MAX_MAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
        public static final double MAX_ROTATIONAL_SLEW_RATE_RPS = 2.0; // percent per second (1 = 100%)

    }
    public static final class ModuleConstants {
        // Wheel Diameter (estimated in meters)
        public static final double WHEEL_DIAMETER = 0.0672;

        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int DRIVE_MOTOR_PINION_TEETH = 13;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
        // bevel pinion
        // tells how much slower the actual wheel spins compared to the motor
        public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_PINION_TEETH * 15);


        public static final double DRIVE_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (Math.PI * WHEEL_DIAMETER 
        * DRIVE_MOTOR_FREE_SPEED_RPS) / DRIVE_MOTOR_REDUCTION;


        // feedforward gain for drive motor. helps predict required motor output for a desired speed
        public static final double DRIVE_VELOCITY_FEEDFOWARD = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;

        // Conversion Factors
        public static final double ROTATIONS_TO_METERS = Math.PI * WHEEL_DIAMETER / DRIVE_MOTOR_REDUCTION;
        public static final double RPM_TO_MPS = Math.PI * WHEEL_DIAMETER / (60 * DRIVE_MOTOR_REDUCTION);

        public static final double ROTATIONS_TO_RADIANS = 2 * Math.PI;
        public static final double RPM_TO_RADPS = 2 * Math.PI / 60.0;

    }
    public static final class NeoMotorConstants {
        // the maximum speed of a neo motor
        public static final double FREE_SPEED_RPM = 5676;
    }
    public static final class ElevatorConstants {
        // CAN IDs
        public static final int ELEVATOR_PRIMARY_MOTOR_ID = 9;
        public static final int ELEVATOR_SECONDARY_MOTOR_ID = 10;
        
        // DIO ports for limit switches
        public static final int ELEVATOR_TOP_LIMIT_SWITCH_ID = 0;
        public static final int ELEVATOR_BOTTOM_LIMIT_SWITCH_ID = 1;
    }
    public static final class ShooterConstants {
        // CAN IDs
        public static final int SHOOTER_PRIMARY_MOTOR_ID = 11; // TODO: NOT FINAL
        public static final int SHOOTER_SECONDARY_MOTOR_ID = 12; // TODO: ALSO NOT FINAL
    }
}