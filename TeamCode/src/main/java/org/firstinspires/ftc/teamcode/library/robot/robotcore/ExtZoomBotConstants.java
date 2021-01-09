package org.firstinspires.ftc.teamcode.library.robot.robotcore;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ExtZoomBotConstants {
    public static boolean ACTIVE_FLYWHEEL = false;
    public static double RING_LOAD_SERVO_PUSH = 0.4; // was 0.5
    public static double RING_LOAD_SERVO_BACK = 0.27; // was 0.18
    public static double LARGE_CHANGE = 100;
    public static int AUTO_SHOOT_WAIT = 350;
    public static int AUTO_POWER_WAIT = 350;
    public static double SMALL_CHANGE = 25;
    public static double ZOOM_POWER = 0.5;
    public static double ZOOM_VELOCITY = 1100;
    public static double STAGE2_POWER_LIMIT = 0.50;
    public static double ZOOM_MODE_VEL_POWER = 0;
    public static boolean ENERGIZE = true;
    public static DcMotor.RunMode ZOOM_MODE = DcMotor.RunMode.RUN_USING_ENCODER;
    public static PIDFCoefficients VELOCITY_PID = new PIDFCoefficients(140, 0, 0, 13.5);

    public static double AUTO_TEST_1 = 800.0;
    public static double AUTO_TEST_2 = 1100.0;
    public static double AUTO_TEST_ANG = 10.0;

    public static double VELO_PRESET_1 = 1000.0;
    public static double VELO_PRESET_2 = 1100.0;
    public static double VELO_PRESET_3 = 1150.0;

    public static RevBlinkinLedDriver.BlinkinPattern VELO_PRESET_1_COLOR = RevBlinkinLedDriver.BlinkinPattern.RED;
    public static RevBlinkinLedDriver.BlinkinPattern VELO_PRESET_2_COLOR = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    public static RevBlinkinLedDriver.BlinkinPattern VELO_PRESET_3_COLOR = RevBlinkinLedDriver.BlinkinPattern.BLUE;

    public static int WOBBLE_RELEASE_DELAY = 0;
}
