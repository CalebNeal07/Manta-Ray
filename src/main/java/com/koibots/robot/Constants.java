// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class CANDeviceIDs {
        public static final int FL_DRIVE = 0;
        public static final int FR_DRIVE = 1;
        public static final int BL_DRIVE = 2;
        public static final int BR_DRIVE = 3;
        public static final int FL_TURN = 4;
        public static final int FR_TURN = 5;
        public static final int BL_TURN = 6;
        public static final int BR_TURN = 7;
        public static final int PIGEON2 = 8;
    }

    public static class ControlConstants {
        public static final double TURN_KP = 0.0;
        public static final double TURN_KD = 0.0;
        public static final double TURN_KS = 0.0;
    }

    public static class PhysicalConstants {
        public static final double DRIVEBASE_WIDTH = Units.inchesToMeters(24);

        public static final Translation2d[] MODULE_LOCATIONS = new Translation2d[] {new Translation2d(
                -Constants.PhysicalConstants.DRIVEBASE_WIDTH / 2,
                Constants.PhysicalConstants.DRIVEBASE_WIDTH / 2),
                            new Translation2d(
                                    Constants.PhysicalConstants.DRIVEBASE_WIDTH / 2,
                                    Constants.PhysicalConstants.DRIVEBASE_WIDTH / 2),
                            new Translation2d(
                                    -Constants.PhysicalConstants.DRIVEBASE_WIDTH / 2,
                                    -Constants.PhysicalConstants.DRIVEBASE_WIDTH / 2),
                            new Translation2d(
                                    Constants.PhysicalConstants.DRIVEBASE_WIDTH / 2,
                                    -Constants.PhysicalConstants.DRIVEBASE_WIDTH / 2)};
        
        public static final double DRIVE_GEAR_RATIO = 0.0;
        public static final double TURN_GEAR_RATIO = 0.0;
    }
}
