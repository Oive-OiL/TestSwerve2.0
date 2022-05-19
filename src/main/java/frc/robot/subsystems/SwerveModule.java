// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule{
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonSRX turningMotor;

    private final Encoder driveEncoder;
    private final Encoder turningEncoder;

    private final PIDController turningPidController; 

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed; 
    private final double absoluteEncoderOffsetRad;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
  int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new AnalogInput(absoluteEncoderId);

    driveMotor = new WPI_TalonFX(driveMotorId);
    turningMotor = new WPI_TalonSRX(turningMotorId);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveEncoder.setPositiveConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversioinFactor(ModuleConstants.kDriveEncoderRPMMeterPerSec);
    turningEncoder.setPositiveConversionFactor(ModuleConstants.kTurningEncoderRot2Rad); 
    turningEncoder.setVelocityConversioinFactor(ModuleConstants.kTunringEncoderRPM2RadPerSec);

    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }

  public double getDrivePosition(){
    return driveEncoder.getPosition(); 
  }

  public double getTurningPosition(){
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity(){
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad(){
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1 : 1);
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state){
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
  }

  public void stop(){
    driveMotor.set(0);
    turningMotor(0);
  }
