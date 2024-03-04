package frc.robot.subsystems;

import static frc.robot.Constants.Drivetrain.*;

import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive;
import com.techhounds.houndutil.houndlib.swerve.NEOCoaxialSwerveModule;
import com.techhounds.houndutil.houndlog.interfaces.Log;

public class Drivetrain implements BaseSwerveDrive {
    @Log(groups = "modules")
    private NEOCoaxialSwerveModule frontLeft = new NEOCoaxialSwerveModule(
            FRONT_LEFT_DRIVE_MOTOR_ID,
            FRONT_LEFT_STEER_MOTOR_ID,
            FRONT_LEFT_STEER_ENCODER_ID,
            false,
            false,
            false,
            FRONT_LEFT_OFFSET,
            SWERVE_CONSTANTS);

    /** The front right swerve module when looking at the bot from behind. */
    @Log(groups = "modules")
    private NEOCoaxialSwerveModule frontRight = new NEOCoaxialSwerveModule(
            FRONT_RIGHT_DRIVE_MOTOR_ID,
            FRONT_RIGHT_STEER_MOTOR_ID,
            FRONT_RIGHT_STEER_ENCODER_ID,
            false,
            false,
            false,
            FRONT_RIGHT_OFFSET,
            SWERVE_CONSTANTS);

    /** The back left swerve module when looking at the bot from behind. */
    @Log(groups = "modules")
    private NEOCoaxialSwerveModule backLeft = new NEOCoaxialSwerveModule(
            BACK_LEFT_DRIVE_MOTOR_ID,
            BACK_LEFT_STEER_MOTOR_ID,
            BACK_LEFT_STEER_ENCODER_ID,
            false,
            false,
            false,
            BACK_LEFT_OFFSET,
            SWERVE_CONSTANTS);

    /** The back right swerve module when looking at the bot from behind. */
    @Log(groups = "modules")
    private NEOCoaxialSwerveModule backRight = new NEOCoaxialSwerveModule(
            BACK_RIGHT_DRIVE_MOTOR_ID,
            BACK_RIGHT_STEER_MOTOR_ID,
            BACK_RIGHT_STEER_ENCODER_ID,
            false,
            false,
            false,
            BACK_RIGHT_OFFSET,
            SWERVE_CONSTANTS);
}
