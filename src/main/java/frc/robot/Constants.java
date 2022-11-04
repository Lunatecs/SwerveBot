// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ModuleConstants {
        public static final double wheelDiameter = 4.0; // 0.102 meters
        public static final double circumference = Math.PI * wheelDiameter;
        public static final double conversionInches = circumference/2048;

        public static final double turningPID_p = 0.0;
        public static final double turningPID_i = 0.0;
        public static final double turningPID_d = 0.0;
        public static final double drivePID_p = 0.0;
        public static final double drivePID_i = 0.0;
        public static final double drivePID_d = 0.0;

        public static final double driveEncoderMultiplier = (Math.PI * 0.102) / 14150; //check this value and change it later
        public static final double turnEncoderMulitplier = (Math.PI * 2) / 360; // check and change if neccesary
     }

    public static final class DrivetrainConstants {
        public static final int driveMotor_LF = 9;
        public static final int turnMotor_LF = 8;

        public static final int driveMotor_RF = 10;
        public static final int turnMotor_RF = 13;

        public static final int driveMotor_LB = 7;
        public static final int turnMotor_LB = 11;

        public static final int driveMotor_RB = 6;
        public static final int turnMotor_RB = 12;

        public static final double driveMaxOutput = 1.0;//Lower as needed

        public static final int canCoderID_LF = 2;
        public static final int canCoderID_LB = 5;
        public static final int canCoderID_RF = 4;
        public static final int canCoderID_RB = 3;

        public static final double encoderOffset_LF = 0;
        public static final double encoderOffset_RF = 0;
        public static final double encoderOffset_LB = 0;
        public static final double encoderOffset_RB = 0;

        public static final double kMaxSpeed = 2;// meters per second
        public static final double maxSpeed = 0.8;
        public static final int pigeonID = 1;

    }

    public static final class ControllerConstants {
        // Logitech controllers
        public static final int JOYSTICK_RIGHT_X_AXIS = 4;
        public static final int JOYSTICK_RIGHT_Y_AXIS = 5;
        public static final int JOYSTICK_LEFT_X_AXIS = 0;
        public static final int JOYSTICK_LEFT_Y_AXIS = 1;

        public static final int Red_Button_ID = 2;
        public static final int Green_Button_ID = 1;
        public static final int Yellow_Button_ID = 4;
        public static final int Blue_Button_ID = 3;

        public static final int Left_Bumper = 5;
        public static final int Right_Bumper = 6;
        public static final int Right_Trigger = 3;
        public static final int Left_Trigger = 2;

        public static final int Joystick_Left_Button = 9;
        public static final int Joystick_Right_Button = 10;

        // Range for inputs that we will consider to be no input
        public static double NoInputTolerance = 0.25;
    }
    public static final class ThrustMasterJoystick{
        //For the big joystick
        public static final int Axis_X = 0;
        public static final int Axis_Y = 1;
        public static final int Axis_Rot = 2;
        public static final int Axis_Throttle = 3;
        public static final int Button_Trigger = 1;
        public static final int Button_Thumb = 2;
        public static final int Button_Thumb_Down = 2;
        public static final int Button_Thumb_Left = 3;
        public static final int Button_Thumb_Right = 4;
        public static final int Button_Left_Left = 5;
        public static final int Button_Left_Middle = 6;
        public static final int Button_Left_Right = 7;
        public static final int Button_Right_Right = 11;
        public static final int Button_Right_Middle = 12;
        public static final int Button_Right_Left = 13;
    }
}
