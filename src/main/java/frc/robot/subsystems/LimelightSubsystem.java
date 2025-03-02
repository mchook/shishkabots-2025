package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable m_limelightTable;
    
    // NetworkTable entries for common Limelight values
    private NetworkTableEntry tx;  // Horizontal offset from crosshair to target
    private NetworkTableEntry ty;  // Vertical offset from crosshair to target
    private NetworkTableEntry ta;  // Target area (0% to 100% of image)
    private NetworkTableEntry tv;  // Whether the limelight has any valid targets (0 or 1)
    
    public LimelightSubsystem() {
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        
        // Initialize NetworkTable entries
        tx = m_limelightTable.getEntry("tx");
        ty = m_limelightTable.getEntry("ty");
        ta = m_limelightTable.getEntry("ta");
        tv = m_limelightTable.getEntry("tv");
        
        // Set default pipeline
        setPipeline(0);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
    /**
     * @return horizontal offset from crosshair to target (-27 degrees to 27 degrees)
     */
    public double getX() {
        return tx.getDouble(0.0);
    }
    
    /**
     * @return vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees)
     */
    public double getY() {
        return ty.getDouble(0.0);
    }
    
    /**
     * @return target area (0% to 100% of image)
     */
    public double getArea() {
        return ta.getDouble(0.0);
    }
    
    /**
     * @return whether the limelight has any valid targets (0 or 1)
     */
    public boolean hasValidTarget() {
        double targetValue = getRawTargetValue();
        return targetValue > 0.5;
    }
    
    /**
     * Sets LED mode
     * @param mode 0 = use pipeline mode, 1 = force off, 2 = force blink, 3 = force on
     */
    public void setLEDMode(int mode) {
        m_limelightTable.getEntry("ledMode").setNumber(mode);
    }
    
    /**
     * Sets camera mode
     * @param mode 0 = vision processor, 1 = driver camera
     */
    public void setCameraMode(int mode) {
        m_limelightTable.getEntry("camMode").setNumber(mode);
    }
    
    /**
     * Sets current pipeline
     * @param pipeline Pipeline index (0-9)
     */
    public void setPipeline(int pipeline) {
        m_limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
    
    /**
     * Sets the exposure time for the camera
     * @param exposure Exposure time in milliseconds (0-100)
     */
    public void setExposure(double exposure) {
        m_limelightTable.getEntry("exposure").setNumber(exposure);
    }

    /**
     * Sets the black level offset
     * @param blackLevel Black level offset (0-100)
     */
    public void setBlackLevel(double blackLevel) {
        m_limelightTable.getEntry("black_level").setNumber(blackLevel);
    }

    /**
     * Gets the raw target detection value from NetworkTables
     * @return raw tv value (0.0 if no target, 1.0 if target detected)
     */
    public double getRawTargetValue() {
        return m_limelightTable.getEntry("tv").getDouble(0.0);
    }

    /**
     * Gets the detected AprilTag ID
     * @return AprilTag ID number, or 0.0 if no tag detected
     */
    public double getTargetID() {
        return m_limelightTable.getEntry("tid").getDouble(0.0);
    }
}
