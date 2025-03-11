package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private SparkMax topMotor;
    private SparkMax bottomMotor;
    private SparkClosedLoopController ClosedLoopController;

    // CONSTANTS
    private final double velocityToShoot = 10.0;
    private final double velocityToHold = 2.0;

    // boolean
    private boolean isHoldingCoral;

    public ShooterSubsystem(int primaryMotorCanId, int secondaryMotorCanId) {
        topMotor = new SparkMax(primaryMotorCanId, MotorType.kBrushless);
        bottomMotor = new SparkMax(secondaryMotorCanId, MotorType.kBrushless);
        ClosedLoopController = topMotor.getClosedLoopController();

        topMotor.configure(Configs.Shooter.primaryMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        isHoldingCoral = false;
    }

    public void holdCoral() {
        ClosedLoopController.setReference(velocityToHold, ControlType.kVelocity);
    }

    public void shoot() {
        // sets the motors to the very fast speed to shoot out the coral
        ClosedLoopController.setReference(velocityToShoot, ControlType.kVelocity);
    }
    public void stop() {
        topMotor.stopMotor();
        bottomMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (isHoldingCoral) {
        bottomMotor.set(-topMotor.get());
        } else {
            bottomMotor.set(topMotor.get());
        }
    }
}
