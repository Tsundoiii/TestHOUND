package frc.robot;

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
    }
}
