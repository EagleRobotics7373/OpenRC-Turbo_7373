package org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

@Config
public class DriveConstantsRingPlace {
    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 0.5;
    public static double TRACK_WIDTH = 14.0;

    public static double MAX_RPM = 1150;
    public static double TICKS_PER_REV = 145.6;

    public static double kV = /*0.0035 .00772*/  /* 1.0 / rpmToVelocity(getMaxRpm())*/ 0.006;
    public static double kA = 0.0;
    public static double kStatic = .014;

    public static double kF = getMotorVelocityF();

    public static Pose2d globalPoseEstimate = null;

    public static boolean RUN_USING_ENCODER = true;
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(40, 0, 20);

    public static DriveConstraints BASE_CONSTRAINTS =
            new DriveConstraints(
//                    50.0, 30.0, 40.0,
                    35.0, 25.0, 40.0,
                    Math.PI, Math.PI, 0.0
            );

    public static PIDCoefficients TRANSLATIONAL_X_PID =
            new PIDCoefficients(4.2, 0.0, 0.4);

    public static PIDCoefficients TRANSLATIONAL_Y_PID =
            new PIDCoefficients(3.5, 0.0, 0.1);

    public static PIDCoefficients HEADING_PID =
            new PIDCoefficients(4, 0.1, 0.05);

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return MAX_RPM * 0.85;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / 145.6;
    }

    public static double getTicksPerSec() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        return MAX_RPM * TICKS_PER_REV / 60.0;
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / getTicksPerSec();
    }
}