package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;

public class Constants {

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.Inches.of(26.5).in(Units.Meters);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.Inches.of(26.5).in(Units.Meters);
    
    // Locations for the swerve drive modules relative to the robot center
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

    public static final int PIGEON_CAN_ID = 8;
    // Spark Max CAN Ids
    public static final int DRIVE_FRONT_LEFT_CAN_ID = 4;
    public static final int DRIVE_BACK_LEFT_CAN_ID = 2;
    public static final int DRIVE_FRONT_RIGHT_CAN_ID = 6;
    public static final int DRIVE_BACK_RIGHT_CAN_ID = 8;

    public static final int DRIVE_TURN_FRONT_LEFT_CAN_ID = 3;
    public static final int DRIVE_TURN_REAR_LEFT_CAN_ID = 1;
    public static final int DRIVE_TURN_FRONT_RIGHT_CAN_ID = 5;
    public static final int DRIVE_TURN_REAR_RIGHT_CAN_ID = 7;


    public static final double MAX_SPEED_IN_MPS = 1.0; // meters per second
    public static final double MAX_ANGULAR_SPEED_IN_RPS = 0.55 * Math.PI; // radians per second

    public static final double MAX_DIRECTION_SLEW_RATE_RPS = 1.3; // radians per second
    public static final double MAX_MAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
    public static final double MAX_ROTATIONAL_SLEW_RATE_RPS = 2.0; // percent per second (1 = 100%)

}
