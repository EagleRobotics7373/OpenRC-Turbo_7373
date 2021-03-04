package org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

@Config
public class DriveConstantsSlowMisumi {
    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 0.5;
    public static double TRACK_WIDTH = 14.0;

    public static double MAX_RPM = 1150;
    public static double TICKS_PER_REV = 145.6;

    public static double kV = /*0.0035 .00772*/  /* 1.0 / rpmToVelocity(getMaxRpm())*/ 0.006;
    public static double kA = 0.0;
    public static double kStatic = .014;

    public static boolean RUN_USING_ENCODER = true;
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(50, 0.7, 30);

    public static DriveConstraints BASE_CONSTRAINTS =
            new DriveConstraints(
//                    50.0, 30.0, 40.0,
                    60.0, 27.5, 40.0,
                    Math.PI, Math.PI, 0.0
            );

    public static PIDCoefficients TRANSLATIONAL_X_PID =
            new PIDCoefficients(4.9, 0.04, 0.45);

    public static PIDCoefficients TRANSLATIONAL_Y_PID =
            new PIDCoefficients(5.5, 0.08, 0.50);

    public static PIDCoefficients HEADING_PID =
            new PIDCoefficients(4.7, 0.2, 0.15);
}