package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule{
    public int moduleNumber;
    private double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private TalonSRX angleEncoder;
    private double lastAngle;
    private SwerveModuleConstants constants;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.constants = moduleConstants;
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new TalonSRX(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            percentOutput = Math.signum(percentOutput) * Math.pow(percentOutput, 2);
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput / Constants.Swerve.speedDivider);
        } else{
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity / 15, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        //double desiredAngle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        double desiredAngle = desiredState.angle.getDegrees();
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(desiredAngle, Constants.Swerve.angleGearRatio));
        if(moduleNumber == 2)
            SmartDashboard.putNumber("Mod 2 angle", desiredAngle);
        lastAngle = desiredAngle;
        SmartDashboard.putNumber("Mod " + this.moduleNumber + " Desired", desiredAngle);

    }

    private void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.Swerve.angleGearRatio);
        Util.checkError(() -> mAngleMotor.setSelectedSensorPosition(absolutePosition, 0, 50), 3);
        if(moduleNumber == 2){
            System.out.println("_____" + getCanCoder().getDegrees());
            System.out.println("_____" + angleOffset);
            System.out.println("_____" + absolutePosition);
        }
    }

    private void configAngleEncoder(){
        Util.checkError(() -> angleEncoder.configFactoryDefault(30), 3);
        Util.checkError(() -> angleEncoder.configFeedbackNotContinuous(true, 30), 3);
        Util.checkError(() -> angleEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30), 3);
        angleEncoder.setSensorPhase(Constants.Swerve.canCoderInvert);
    }

    private void configAngleMotor(){
        Util.checkError(() -> mAngleMotor.configFactoryDefault(30), 3);
        Util.checkError(() -> mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig, 30), 3);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        Util.checkError(() -> mAngleMotor.configRemoteFeedbackFilter(angleEncoder.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, 0, 30), 3);
        Util.checkError(() -> mAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30), 3);
        resetToAbsolute();
    }

    private void configDriveMotor(){
        Util.checkError(() -> mDriveMotor.configFactoryDefault(30), 3);
        Util.checkError(() -> mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig, 30), 3);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        Util.checkError(() -> mDriveMotor.setSelectedSensorPosition(0, 0, 30), 3);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(Conversions.MagToDegrees(angleEncoder.getSelectedSensorPosition()));
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

}