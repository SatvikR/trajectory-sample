// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final double ksVolts = 0.59099;
        public static final double kvVoltSecondsPerMeter = 3.9611;
        public static final double kaVoltSecondsSquaredPerMeter = 0.31905;
        public static final double kPDriveVel = 4.6334;

        public static final double kTrackWidthMeters = Units.inchesToMeters(23);
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackWidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        public static final int leftUpperMotorID = 4;
        public static final int leftLowerMotorID = 3;
        public static final int rightLowerMotorID = 1;
        public static final int rightUpperMotorID = 2;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(7.52);
        public static final double kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(4);

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
