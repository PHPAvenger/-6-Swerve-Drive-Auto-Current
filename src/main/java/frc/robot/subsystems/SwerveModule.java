package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    //declare the spark motor controllers
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    //The encoders inside the neos
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;
    //The thirfty bot encoder for absolute values. This will let us know the angle of the wheel
    private final AnalogEncoder absoluteEncoder;
    //if the module is reversed or not.
    private final boolean absoluteEncoderReversed;
    //The offset of the motor
    private final double absoluteEncoderOffsetRad;  

    private final PIDController turningPidController;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogEncoder(absoluteEncoderId);

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

       

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

      
        //Spark encoders can only be configured using the new SparkMaxConfig class
        //
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        //driveMotor.setInverted(driveMotorReversed);
        driveConfig.inverted(driveMotorReversed);

        //old way of setting conversion factors
        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        
        //using the config object, we access the encoder property
        //then the conversion factor for position and velocity
        driveConfig.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
        .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        //once the config has been set we can call the configure method for the drive motor 
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //config for the turninging motor
        SparkMaxConfig turningConfig = new SparkMaxConfig();
        turningConfig.inverted(turningMotorReversed);
        //turningMotor.setInverted(turningMotorReversed);
        //turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        turningConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
        .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        //Set the config for the turning motor using the config object
        turningMotor.configure(turningConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters );
        
        //We use the wpi pid controller to control how the the wheels will turn.
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        //we want the PID to turn the wheel correctly
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }
    //This method will determine the actual angle by taking the reading and dividing the voltage
    //supplied to the absolute encoder
    //we then substract the offset and return the angle
    public double getAbsoluteEncoderRad() {
        // old code double angle = absoluteEncoder.get() * Math.PI / 50;
        double angle = absoluteEncoder.get() * 360 * ( Math.PI / 180);
        // angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);

    }
    //we reset the encoders when the robot boots up
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    //The state is a combination of the speed and the current angle of the module
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    @SuppressWarnings("deprecation")
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        //state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("PID[" + absoluteEncoder.getChannel() + "]", getTurningPosition() + " " +state.angle.getRadians());
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
