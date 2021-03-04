package org.firstinspires.ftc.teamcode.library.vision.ultimategoal;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.library.functions.AllianceColor;

@Config
public class UltimateGoalPowerShotConstants {

    public static AllianceColor ALLIANCE_COLOR = AllianceColor.RED;
    public static int TARGET_X = 420;

    public static int GET_MODIFIED_TARGET_X() {
        return TARGET_X - CUTOFF_LEFT;
    }

    public static int TARGET_PS = 1;
    public static double SAFE_SHOT = 0.75;

    public static int CUTOFF_TOP = 425;
    public static int CUTOFF_BOTTOM = 550;
    public static int CUTOFF_LEFT = 100;
    public static int CUTOFF_RIGHT = 800;


    public static int RED_HUE_LOW = 6;
    public static int RED_HUE_HIGH = 170;
    public static int RED_SAT_LOW = 140;
    public static int RED_SAT_HIGH = 245;

    public static int BLUE_HUE_LOW = 110;
    public static int BLUE_HUE_HIGH = 130;
    public static int BLUE_SAT_LOW = 100;
    public static int BLUE_SAT_HIGH = 220;

    public static int VALUE_LOW = 30;
    public static int VALUE_HIGH = 255;

    public static int AREA_CUTOFF = 450;
    public static int NUM_SIDES_CUTOFF = 12;
    public static double ASPECT_LOW_CUTOFF = 0.1;
    public static double ASPECT_HIGH_CUTOFF = 0.7;

    public static double BLUR_WIDTH = 2.0;
    public static double BLUR_HEIGHT = 2.0;
    public static int BLUR_MEDIAN_SIZE = 1;
    public static PixelAction BLUR_TYPE = PixelAction.NONE;

    public static int KERNEL_WIDTH = 0;
    public static int KERNEL_HEIGHT = 3;
    public static double CONTOUR_APPROX = 0.04;

    public static boolean DRAW_CONTOURS = false;
    public static int OUTPUT_INDEX = 0;

    public static PixelAction PIXEL_EXPANSION_ACTION = PixelAction.DILATE;

    public static boolean RETURN_SINGLE_CHANNEL = false;

    public enum PixelAction {
        ERODE, DILATE, NONE, BLUR, BLUR_MEDIAN
    }

}
