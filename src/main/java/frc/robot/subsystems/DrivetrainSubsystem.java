// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  SwerveModule leftFront = new SwerveModule(DrivetrainConstants.driveMotor_LF, DrivetrainConstants.turnMotor_LF, DrivetrainConstants.canCoderID_LF, DrivetrainConstants.encoderOffset_LF);
  SwerveModule rightFront = new SwerveModule(DrivetrainConstants.driveMotor_LB, DrivetrainConstants.turnMotor_LB, DrivetrainConstants.canCoderID_LB, DrivetrainConstants.encoderOffset_LB);
  SwerveModule leftBack = new SwerveModule(DrivetrainConstants.driveMotor_RF, DrivetrainConstants.turnMotor_RF, DrivetrainConstants.canCoderID_RF, DrivetrainConstants.encoderOffset_RF);
  SwerveModule rightBack = new SwerveModule(DrivetrainConstants.driveMotor_RB, DrivetrainConstants.turnMotor_RB, DrivetrainConstants.canCoderID_RB, DrivetrainConstants.encoderOffset_RB);
  
  private final PigeonIMU gyro = new PigeonIMU(DrivetrainConstants.pigeonID);
  private final Translation2d leftFrontLocation = new Translation2d(.3, .3);
  private final Translation2d rightFrontLocation = new Translation2d(.3, -.3);
  private final Translation2d rightBackLocation = new Translation2d(-.3, .3);
  private final Translation2d leftBackLocation = new Translation2d(-.3, -.3);

  private double maxSpeed = DrivetrainConstants.maxSpeed;

  public enum Module {
    leftFrontLocation, rightFrontLocation, leftBackLocation, rightBackLocation;
  }

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontLocation, rightFrontLocation, leftBackLocation, rightBackLocation);
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getAngle());
  private boolean isFieldRelative = true;
  private boolean isXDefault = false;


  public DrivetrainSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

//  public void CanDrive(boolean value){
//    leftFront.canDrive(value);
//    rightFront.canDrive(value);
//    leftBack.canDrive(value);
//    rightBack.canDrive(value);
//  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees((gyro.getFusedHeading())); //DO PIGEON SHIT !!!BAD WORD >:(!!!
  }

  public void zeroGyroHeading() {
    gyro.setFusedHeading(0.0);
  }

  public void setGyro(double degrees) {
    gyro.setFusedHeading(degrees);
  }

  public boolean IsFieldRelative() {
    return this.isFieldRelative; //TODO: Decide whether or not to add robot relative
  }

  public void SetFieldRelative(boolean newValue) {
    this.isFieldRelative = newValue;
  }

  public boolean isXDefault() {
    return isXDefault;
  }

  public void setXDefault(boolean value) {
    isXDefault = value;
  }

  public void setMaxSpeed(double value) {
    maxSpeed = DrivetrainConstants.maxSpeed + value;
  }

  public double getMaxSpeed() {
    return maxSpeed;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = kinematics
        .toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

    leftFront.setDesiredState(swerveModuleStates[0]);
    rightFront.setDesiredState(swerveModuleStates[1]);
    leftBack.setDesiredState(swerveModuleStates[2]);
    rightBack.setDesiredState(swerveModuleStates[3]);
  }

  // private ChassisSpeeds fromFieldRelativeSpeeds(double vxMetersPerSecond, double vyMetersPerSecond,
  //     double omegaRadiansPerSecond, Rotation2d robotAngle) {
  //   return new ChassisSpeeds(vxMetersPerSecond * robotAngle.getCos() - vyMetersPerSecond * robotAngle.getSin(),
  //       vxMetersPerSecond * robotAngle.getSin() + vyMetersPerSecond * robotAngle.getCos(), omegaRadiansPerSecond);
  // }

  public void setWheelAngleStates(double fl, double fr, double bl, double br) {
    setWheelState(Module.leftFrontLocation, fl, 0.0);
    setWheelState(Module.rightFrontLocation, fr, 0.0);
    setWheelState(Module.leftBackLocation, bl, 0.0);
    setWheelState(Module.rightBackLocation, br, 0.0);
  }

  public double[] getWheelAngles(){
    double[] wheelAngles = new double[] {getWheelState(Module.leftFrontLocation).angle.getDegrees(), getWheelState(Module.rightFrontLocation).angle.getDegrees(),
      getWheelState(Module.leftBackLocation).angle.getDegrees(), getWheelState(Module.rightBackLocation).angle.getDegrees()};
    return wheelAngles;
  }

  public void setWheelState(Module module, double angle, double speed) {
    angle = Math.toRadians(angle);

    switch (module) {
    case leftFrontLocation:
      leftFront.setDesiredState(new SwerveModuleState(speed, new Rotation2d(angle)));
      break;
    case rightFrontLocation:
      rightFront.setDesiredState(new SwerveModuleState(speed, new Rotation2d(angle)));
      break;
    case leftBackLocation:
      leftBack.setDesiredState(new SwerveModuleState(speed, new Rotation2d(angle)));
      break;
    case rightBackLocation:
      rightBack.setDesiredState(new SwerveModuleState(speed, new Rotation2d(angle)));
      break;
    }
  }

  public double getWheelDriveEncoder(Module module) {
    switch (module) {
    case leftFrontLocation:
      return leftFront.GetDriveEncoder();
    case rightFrontLocation:
      return rightFront.GetDriveEncoder();
    case leftBackLocation:
      return leftBack.GetDriveEncoder();
    case rightBackLocation:
      return rightBack.GetDriveEncoder();
    default:
      return 0;
    }
  }

  public SwerveModuleState getWheelState(Module module) {
    switch (module) {
    case leftFrontLocation:
      return leftFront.getState();
    case rightFrontLocation:
      return rightFront.getState();
    case leftBackLocation:
      return leftBack.getState();
    case rightBackLocation:
      return rightBack.getState();
    default:
      return null;
    }
  }

  public void resetAllDriveEncoders() {
    leftFront.resetDriveEncoder();
    rightFront.resetDriveEncoder();
    leftBack.resetDriveEncoder();
    rightBack.resetDriveEncoder();
  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    odometry.update(getAngle(), leftFront.getState(), rightFront.getState(), leftBack.getState(),
        rightBack.getState());
  }
  public static double getCurve(double input) { // slopes up the value of speed in order to prevent jumpiness when driving
    double sign = Math.signum(input);

    double value = Math.abs(input);
    value = Math.pow(value, 2);
    value += 0.02;

    return input; //TODO: edit the value for getCurve as necessary
  } 
}

