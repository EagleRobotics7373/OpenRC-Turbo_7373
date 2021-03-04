package org.firstinspires.ftc.teamcode.library.robot.robotcore;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
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
    public static double PS_ROTATE_SPEED = 0.3;
    public static boolean PS_FRONT_LEFT = true;
    public static boolean PS_FRONT_RIGHT = true;
    public static boolean PS_BACK_LEFT = false;
    public static boolean PS_BACK_RIGHT = false;
    public static double PS_AUTO_ROTATE_SAFE = 0.7;
    public static PIDCoefficients PS_SEEK = new PIDCoefficients(0.07, 0.0, 0.0);
    public static double PS_SEEK_MIN = 0.15;
    public static double PS_SEEK_MAX = 0.2;
    public static double ZOOM_MODE_VEL_POWER = 0;
    public static boolean ENERGIZE = true;
    public static DcMotor.RunMode ZOOM_MODE = DcMotor.RunMode.RUN_USING_ENCODER;
    public static PIDFCoefficients VELOCITY_PID = new PIDFCoefficients(140, 0, 0, 13.5);

    public static double SERVO_DEFLECTION_POS = 0.50;

    public static double AUTO_TEST_1 = 800.0;
    public static double AUTO_TEST_2 = 1000.0;
    public static double AUTO_TEST_ANG = 0.08;

    public static double AUTO_SERVO_POS_1 = 0.60;
    public static double AUTO_SERVO_SPEED_1 = 1200;
    public static double AUTO_SERVO_POS_2 = 0.60;
    public static double AUTO_SERVO_SPEED_2 = 1200;
    public static double AUTO_SERVO_POS_3 = 0.60;
    public static double AUTO_SERVO_SPEED_3 = 1250;
    public static double AUTO_SERVO_WAIT = 500;

    public static double VELO_PRESET_1 = 1000.0;
    public static double VELO_PRESET_2 = 1100.0;
    public static double VELO_PRESET_3 = 1150.0;

    public static RevBlinkinLedDriver.BlinkinPattern VELO_PRESET_1_COLOR = RevBlinkinLedDriver.BlinkinPattern.RED;
    public static RevBlinkinLedDriver.BlinkinPattern VELO_PRESET_2_COLOR = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    public static RevBlinkinLedDriver.BlinkinPattern VELO_PRESET_3_COLOR = RevBlinkinLedDriver.BlinkinPattern.BLUE;

    public static int WOBBLE_RELEASE_DELAY = 0;
}
