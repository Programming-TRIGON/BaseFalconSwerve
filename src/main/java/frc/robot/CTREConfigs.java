package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public final class CTREConfigs{
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;


    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.Swerve.angleEnableCurrentLimit,
                Constants.Swerve.angleContinuousCurrentLimit,
                Constants.Swerve.anglePeakCurrentLimit,
                Constants.Swerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveAngleFXConfig.openloopRamp = Constants.Swerve.angleOpenLoopRamp;
        swerveAngleFXConfig.closedloopRamp = Constants.Swerve.angleClosedLoopRamp;


        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.Swerve.driveEnableCurrentLimit,
                Constants.Swerve.driveContinuousCurrentLimit,
                Constants.Swerve.drivePeakCurrentLimit,
                Constants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = Constants.Swerve.driveOpenLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.Swerve.driveClosedLoopRamp;


    }

}