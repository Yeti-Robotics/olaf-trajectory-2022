// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        public static final int LEFT_FALCON_1 = 1;
        public static final int LEFT_FALCON_2 = 2;
        public static final int RIGHT_FALCON_1 = 3;
        public static final int RIGHT_FALCON_2 = 4;
    
        public static final double WHEEL_DIAMETER = 3.875; //in
        public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;
    
        public static final double HIGH_GEAR_RATIO = 6.86; //jvn  //5.533243;// fudge values
        public static final double LOW_GEAR_RATIO = 15.8839779006; //fudge value //9.93;//jvn // fudge values 
        public static final double DISTANCE_PER_PULSE = (WHEEL_DIAMETER * Math.PI) / 2048; //wheel diam in inches & falcon CPR
        
        public static final int[] SHIFTER_SOLENOID = {1,6}; 
        public static final int GYRO_ID = 13;
    
        public static final double MAX_SPEED_INCHES_PER_SEC = 9.08 * 12.0;
        public static final double MAX_ACCEL_INCHES_PER_SEC2 = 7.5 * 12.0;
    }

}
