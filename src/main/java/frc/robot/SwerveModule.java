// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

public class SwerveModule {
  // Motor value initialization
  private final WPI_TalonFX driveMotor; //add final to these later
  private final WPI_TalonFX turningMotor;
  
  // Encoder Values initialization
  private final CANCoder turnEncoder;
  private double absoluteEncoderOffset; // all the way down here as well

  private final PIDController turningPID = new PIDController(ModuleConstants.turningPID_p, ModuleConstants.turningPID_i, ModuleConstants.turningPID_d);
  private final PIDController drivePidController = new PIDController(ModuleConstants.drivePID_p, ModuleConstants.drivePID_i, ModuleConstants.drivePID_d);
  private boolean isXDefault = false;

  public SwerveModule(int driveMotorID, int turningMotorID, int absoluteEncoderID, double absoluteEncoderOffset) {

    turnEncoder = new CANCoder(absoluteEncoderID);
    
    driveMotor = new WPI_TalonFX(driveMotorID);
    driveMotor.configFactoryDefault();
    driveMotor.setNeutralMode(NeutralMode.Brake);

    turningMotor = new WPI_TalonFX(turningMotorID);
    turningMotor.configFactoryDefault();
    turningMotor.setNeutralMode(NeutralMode.Coast);
    
    resetDriveEncoders();

    turningPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void resetDriveEncoders() {
    driveMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  public void absoluteEncoderOffset() {
    turnEncoder.configMagnetOffset(absoluteEncoderOffset);
  }

  public void resetDriveEncoder() {
    driveMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  public double GetDriveEncoder() {
    return driveMotor.getSensorCollection().getIntegratedSensorPosition();
  }

  public double getAbsolutePosition() {
    return turnEncoder.getAbsolutePosition();
  }

  public boolean isXDefault() {
    return this.isXDefault;
  }

  public void setXDefault(boolean value) {
    this.isXDefault = value;
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getSensorCollection().getIntegratedSensorVelocity(),
        new Rotation2d(Math.toRadians(turnEncoder.getAbsolutePosition())));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    // Changing base encoder velocity to meters per second
    double driveEncoderVelocity = ((driveMotor.getSensorCollection().getIntegratedSensorVelocity())
        * ModuleConstants.driveEncoderMultiplier);
    double driveOutput = drivePidController.calculate(driveEncoderVelocity, state.speedMetersPerSecond);

    // Clamps the drive output to prevent damage to gears
    if (driveOutput > DrivetrainConstants.driveMaxOutput) {
      driveOutput = Constants.DrivetrainConstants.driveMaxOutput;
    } else if (driveOutput < -Constants.DrivetrainConstants.driveMaxOutput) {
      driveOutput = -Constants.DrivetrainConstants.driveMaxOutput;
    }

    //System.out.println(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    // Changing encoder value to radians between -pi to pi
    double turnEncoderPosition = (turnEncoder.getAbsolutePosition()
        * Constants.ModuleConstants.turnEncoderMulitplier) - Math.PI;
    double turnOutput = turningPID.calculate(turnEncoderPosition, state.angle.getRadians());

    SmartDashboard.putNumber("Desired Position", state.angle.getRadians());
    SmartDashboard.putNumber("Current Position", turnEncoderPosition);

//    // Clamps the turn output to prevent damage to gears
//    if (turnOutput > Constants.DrivetrainConstants.turnMaxOutput) {
//      turnOutput = Constants.DrivetrainConstants.turnMaxOutput;
//    } else if (turnOutput < -Constants.DrivetrainConstants.turnMaxOutput) {
//      turnOutput = -Constants.DrivetrainConstants.turnMaxOutput;
//    }

    // TODO: Temporarily commented out
    //System.out.print("SET Output");
    //System.out.print(driveOutput);
    //System.out.println(turnOutput);

    driveOutput = MathUtil.clamp(driveOutput, -0.5, 0.5);

    // Set motor power to pid loop outputs
    driveMotor.set(-driveOutput);
    turningMotor.set(-turnOutput);
  }

}
