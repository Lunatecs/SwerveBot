// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class DriveWithJoysticksCommand extends CommandBase {
  
  DrivetrainSubsystem swerve;
  Joystick controller = new Joystick(0);


  public DriveWithJoysticksCommand(DrivetrainSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = (-controller.getRawAxis(Constants.ThrustMasterJoystick.Axis_X)) * Constants.DrivetrainConstants.kMaxSpeed;
      //gets the forward speed of the robot
    double strafeSpeed = (-controller.getRawAxis(Constants.ThrustMasterJoystick.Axis_Y)) * Constants.DrivetrainConstants.kMaxSpeed;
      //gets the strafing speed of the robot (left to right)
    double rot = -controller.getRawAxis(Constants.ThrustMasterJoystick.Axis_Rot) * Constants.DrivetrainConstants.kMaxAngularSpeed;
    if (Math.abs(strafeSpeed) < 0.15){ // TODO: potentially change or remove the values in the if statement
      strafeSpeed = 0;
      //if a very small input is detected, it will be regarded as controller drift
    }
    System.out.println("ForwardSpeed: "+forwardSpeed);
    System.out.println("StrafeSpeed: "+strafeSpeed);
    System.out.println("Rotation: "+rot);

    double throttle = (-controller.getRawAxis(Constants.ThrustMasterJoystick.Axis_Throttle) + 1);
    swerve.setMaxSpeed(throttle);

    if ((Math.abs(forwardSpeed) < Constants.ControllerConstants.NoInputTolerance)
                    && (Math.abs(strafeSpeed) < Constants.ControllerConstants.NoInputTolerance)
                    && (Math.abs(rot) < Constants.ControllerConstants.NoInputTolerance)) {
      if(swerve.isXDefault()){
        //swerve.CanDrive(true); TODO: figure out what to do with CanDrive method
        swerve.setWheelAngleStates(45, -45, -45, 45);
      }else {
        //swerve.CanDrive(false);
        swerve.drive(0, 0, 0, true);
      }
    }else {
      // swerve.CanDrive(true);
      swerve.drive(DrivetrainSubsystem.getCurve(forwardSpeed), DrivetrainSubsystem.getCurve(strafeSpeed), DrivetrainSubsystem.getCurve(rot), true);
      //code requires asking about fieldrelativity. I set it so fieldrelative is hardcoded on
    }

    //SmartDashboard.putNumber("Angle", )
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
