// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER = 0;  
        public static final int OPERATOR_CONTROLLER = 1;  
    }
    public static class DrivetrainConstants {
        public static final int MOTOR_LEFT_ID = 2;
        public static final int MOTOR_RIGHT_ID = 4;
        public static final double GEAR_RATIO = 1/12.75;
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(6) * Math.PI;
        public static final double ENCODER_CONVERSION_FACTOR = GEAR_RATIO * WHEEL_CIRCUMFERENCE;
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.875);
    }
}