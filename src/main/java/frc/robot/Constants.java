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
        //public static final double conversionVelocity; *Hasn't been finished yet

        public static final double turningPID_p = 0.0;
        public static final double turningPID_i = 0.0;
        public static final double turningPID_d = 0.0;
        public static final double drivePID_p = 0.0;
        public static final double drivePID_i = 0.0;
        public static final double drivePID_d = 0.0;

        public static final double driveEncoderMultiplier = (Math.PI * 0.102) / 14150; //check this value and change it later
        public static final double turnEncoderMulitplier = (Math.PI * 2) / 360; // check and change if neccesary
     }

    public static final class DrivetrainConstants {// sujay plz kill yourself
        public static final int driveMotor_LF = 0;
        public static final int turnMotor_LF = 0;

        public static final int driveMotor_RF = 0;
        public static final int turnMotor_RF = 0;

        public static final int driveMotor_LB = 0;
        public static final int turnMotor_LB = 0;

        public static final int driveMotor_RB = 0;
        public static final int turnMotor_RB = 0;

        public static final double driveMaxOutput = 1.0;//Lower as needed

        public static final int canCoderID_LF = 0;
        public static final int canCoderID_LB = 0;
        public static final int canCoderID_RF = 0;
        public static final int canCoderID_RB = 0;

        public static final double encoderOffset_LF = 0;
        public static final double encoderOffset_RF = 0;
        public static final double encoderOffset_LB = 0;
        public static final double encoderOffset_RB = 0;



        public static final double maxSpeed = 1000000000;
        public static final int pigeonID = 24; // TODO: Change the CAN ID on the Pigeon

    }
}
