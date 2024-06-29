package frc.robot.subsystems;

import static frc.robot.Constants.Drivetrain.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive;
import com.techhounds.houndutil.houndlib.swerve.NEOCoaxialSwerveModule;
import com.techhounds.houndutil.houndlog.interfaces.Log;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase implements BaseSwerveDrive {
    public class Constants {
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

        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.75);
        /** Distance between front and back wheels. */
        public static final double WHEEL_BASE = Units.inchesToMeters(22.75);
        public static final Translation2d[] SWERVE_DRIVE_LOCATIONS = new Translation2d[] {
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        };
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(SWERVE_DRIVE_LOCATIONS[0],
                SWERVE_DRIVE_LOCATIONS[1], SWERVE_DRIVE_LOCATIONS[2], SWERVE_DRIVE_LOCATIONS[3]);
    }

    /** The front left swerve module when looking at the bot from behind. */
    @Log(groups = "modules")
    private NEOCoaxialSwerveModule frontLeft = new NEOCoaxialSwerveModule(
            FrontLeft.DRIVE_MOTOR_ID,
            FrontLeft.STEER_MOTOR_ID,
            FrontLeft.STEER_ENCODER_ID,
            FrontLeft.DRIVE_MOTOR_INVERTED,
            FrontLeft.STEER_MOTOR_INVERTED,
            FrontLeft.STEER_ENCODER_INVERTED,
            FrontLeft.MAGNET_OFFSET,
            SWERVE_CONSTANTS);

    /** The front right swerve module when looking at the bot from behind. */
    @Log(groups = "modules")
    private NEOCoaxialSwerveModule frontRight = new NEOCoaxialSwerveModule(
            FrontRight.DRIVE_MOTOR_ID,
            FrontRight.STEER_MOTOR_ID,
            FrontRight.STEER_ENCODER_ID,
            FrontRight.DRIVE_MOTOR_INVERTED,
            FrontRight.STEER_MOTOR_INVERTED,
            FrontRight.STEER_ENCODER_INVERTED,
            FrontRight.MAGNET_OFFSET,
            SWERVE_CONSTANTS);

    /** The back left swerve module when looking at the bot from behind. */
    @Log(groups = "modules")
    private NEOCoaxialSwerveModule backLeft = new NEOCoaxialSwerveModule(
            BackLeft.DRIVE_MOTOR_ID,
            BackLeft.STEER_MOTOR_ID,
            BackLeft.STEER_ENCODER_ID,
            BackLeft.DRIVE_MOTOR_INVERTED,
            BackLeft.STEER_MOTOR_INVERTED,
            BackLeft.STEER_ENCODER_INVERTED,
            BackLeft.MAGNET_OFFSET,
            SWERVE_CONSTANTS);

    /** The back right swerve module when looking at the bot from behind. */
    @Log(groups = "modules")
    private NEOCoaxialSwerveModule backRight = new NEOCoaxialSwerveModule(
            BackRight.DRIVE_MOTOR_ID,
            BackRight.STEER_MOTOR_ID,
            BackRight.STEER_ENCODER_ID,
            BackRight.DRIVE_MOTOR_INVERTED,
            BackRight.STEER_MOTOR_INVERTED,
            BackRight.STEER_ENCODER_INVERTED,
            BackRight.MAGNET_OFFSET,
            SWERVE_CONSTANTS);

    private SwerveModuleState[] commandedModuleStates = new SwerveModuleState[] { new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    @Override
    public DriveMode getDriveMode() {
        return driveMode;
    }

    @Override
    public Pose2d getPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPose'");
    }

    @Override
    public Rotation2d getRotation() {
        return gyro.getRotation2d();
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getChassisSpeeds'");
    }

    @Override
    public SwerveDrivePoseEstimator getPoseEstimator() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPoseEstimator'");
    }

    @Override
    public void updatePoseEstimator() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updatePoseEstimator'");
    }

    @Override
    public void resetPoseEstimator(Pose2d pose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPoseEstimator'");
    }

    @Override
    public void resetGyro() {
        gyro.reset();
    }

    @Override
    public void setMotorHoldModes(MotorHoldMode motorHoldMode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setMotorHoldModes'");
    }

    @Override
    public void setDriveCurrentLimit(int currentLimit) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDriveCurrentLimit'");
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public void setStates(SwerveModuleState[] state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setStates'");
    }

    @Override
    public void setStatesClosedLoop(SwerveModuleState[] state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setStatesClosedLoop'");
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'drive'");
    }

    @Override
    public void drive(ChassisSpeeds speeds, DriveMode driveMode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'drive'");
    }

    @Override
    public void driveClosedLoop(ChassisSpeeds speeds, DriveMode driveMode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'driveClosedLoop'");
    }

    @Override
    public Command teleopDriveCommand(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier thetaSpeedSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'teleopDriveCommand'");
    }

    @Override
    public Command controlledRotateCommand(DoubleSupplier angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'controlledRotateCommand'");
    }

    @Override
    public Command disableControlledRotateCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'disableControlledRotateCommand'");
    }

    @Override
    public Command wheelLockCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'wheelLockCommand'");
    }

    @Override
    public Command turnWheelsToAngleCommand(double angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'turnWheelsToAngleCommand'");
    }

    @Override
    public Command driveToPoseCommand(Supplier<Pose2d> pose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'driveToPoseCommand'");
    }

    @Override
    public Command followPathCommand(PathPlannerPath path) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'followPathCommand'");
    }

    @Override
    public Command driveDeltaCommand(Transform2d delta, PathConstraints constraints) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'driveDeltaCommand'");
    }

    @Override
    public Command setDriveModeCommand(DriveMode driveMode) {
        return runOnce(() -> this.driveMode = driveMode);
    }

    @Override
    public Command resetGyroCommand() {
        return runOnce(() -> resetGyro());
    }

    @Override
    public Command setDriveCurrentLimitCommand(int currentLimit) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDriveCurrentLimitCommand'");
    }

    @Override
    public Command coastMotorsCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'coastMotorsCommand'");
    }
}
