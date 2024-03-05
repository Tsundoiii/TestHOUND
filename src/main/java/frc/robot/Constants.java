package frc.robot;

import com.techhounds.houndutil.houndlib.swerve.CoaxialSwerveModule.SwerveConstants;

import edu.wpi.first.math.system.plant.DCMotor;

public class Constants {
    public static final class Drivetrain {
        public static final class FrontLeft {
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int STEER_MOTOR_ID = 1;
            public static final int STEER_ENCODER_ID = 1;

            public static final boolean DRIVE_MOTOR_INVERTED = false;
            public static final boolean STEER_MOTOR_INVERTED = false;
            public static final boolean STEER_ENCODER_INVERTED = false;

            public static final double MAGNET_OFFSET = 0.23583984375;
        }

        public static final class FrontRight {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int STEER_MOTOR_ID = 3;
            public static final int STEER_ENCODER_ID = 2;

            public static final boolean DRIVE_MOTOR_INVERTED = false;
            public static final boolean STEER_MOTOR_INVERTED = false;
            public static final boolean STEER_ENCODER_INVERTED = false;

            public static final double MAGNET_OFFSET = 0.138916015625;
        }

        public static final class BackLeft {
            public static final int DRIVE_MOTOR_ID = 6;
            public static final int STEER_MOTOR_ID = 5;
            public static final int STEER_ENCODER_ID = 3;

            public static final boolean DRIVE_MOTOR_INVERTED = false;
            public static final boolean STEER_MOTOR_INVERTED = false;
            public static final boolean STEER_ENCODER_INVERTED = false;

            public static final double MAGNET_OFFSET = 0.059814453125;
        }

        public static final class BackRight {
            public static final int DRIVE_MOTOR_ID = 8;
            public static final int STEER_MOTOR_ID = 7;
            public static final int STEER_ENCODER_ID = 4;

            public static final boolean DRIVE_MOTOR_INVERTED = false;
            public static final boolean STEER_MOTOR_INVERTED = false;
            public static final boolean STEER_ENCODER_INVERTED = false;

            public static final double MAGNET_OFFSET = -0.26953125;
        }

        public static final SwerveConstants SWERVE_CONSTANTS = new SwerveConstants();
        static {
            SWERVE_CONSTANTS.DRIVE_kP = 2.9646;
            SWERVE_CONSTANTS.DRIVE_kI = 0.0;
            SWERVE_CONSTANTS.DRIVE_kD = 0.0;
            SWERVE_CONSTANTS.DRIVE_kS = 0.0;
            SWERVE_CONSTANTS.DRIVE_kV = 2.8024;
            SWERVE_CONSTANTS.DRIVE_kA = 0.057223;
            SWERVE_CONSTANTS.STEER_kP = 6.0;
            SWERVE_CONSTANTS.STEER_kI = 0.0;
            SWERVE_CONSTANTS.STEER_kD = 0.1;

            SWERVE_CONSTANTS.DRIVE_GEARING = 6.12;
            SWERVE_CONSTANTS.STEER_GEARING = 150.0 / 7.0;
            SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * 0.0478;
            SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS = SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE
                    / SWERVE_CONSTANTS.DRIVE_GEARING;
            SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI
                    / SWERVE_CONSTANTS.STEER_GEARING;

            SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 4.282;
            SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
            SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND = 8.829 * Math.PI;
            SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 8.829 * 3 * Math.PI;

            SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT = 50;
            SWERVE_CONSTANTS.STEER_CURRENT_LIMIT = 20;
            SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR = DCMotor.getNEO(1);
            SWERVE_CONSTANTS.STEER_GEARBOX_REPR = DCMotor.getNEO(1);
            SWERVE_CONSTANTS.DRIVE_MOI = 0.04;
            SWERVE_CONSTANTS.STEER_MOI = 0.025;
        }
    }
}
