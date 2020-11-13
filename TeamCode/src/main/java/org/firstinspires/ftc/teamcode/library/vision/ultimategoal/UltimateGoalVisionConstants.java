package org.firstinspires.ftc.teamcode.library.vision.ultimategoal;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.teamcode.library.vision.base.ImageResolution;

@Config
public class UltimateGoalVisionConstants {
    /*

     CONSTANTS FOR PIXEL ANALYSIS PIPELINE

     */

    // (x,y) position of the lowest ring in the stack
    public static int BOTTOM_RING_CENTER_X_ON_LEFT = 80;
    public static int BOTTOM_RING_CENTER_X_ON_RIGHT = 500;
    public static int BOTTOM_RING_CENTER_Y_ON_RIGHT = 165;
    public static int BOTTOM_RING_CENTER_Y_ON_LEFT = 155;

    public static int BOTTOM_RING_CENTER_X = 285;
    public static int BOTTOM_RING_CENTER_Y = 400;


    // Width and height of the ring
    public static int RING_WIDTH = 240;
    public static int RING_HEIGHT = 25;


    // Spacing between rings
    public static int RING_SPACING = 150;

    // Amount of pixels to skip within each ring
    public static int SEARCH_SKIP = 15;

    // Method by which to search each bounding box
    public static RingPixelAnalysisPipeline.SearchType SEARCH_TYPE =
            RingPixelAnalysisPipeline.SearchType.AVERAGE;

    /*

     CONSTANTS FOR CONTOUR DISCOVERY PIPELINE

     */

    // Lower and upper bounds for cv::inRange() function
    public static double CONTOUR_HUE_LOWER_BOUND = 10;
    public static double CONTOUR_HUE_UPPER_BOUND = 25;
    public static double CONTOUR_SAT_LOWER_BOUND = 150;
    public static double CONTOUR_SAT_UPPER_BOUND = 255;
    public static double CONTOUR_LUM_LOWER_BOUND = 0;
    public static double CONTOUR_LUM_UPPER_BOUND = 255;
    public static int CONTOUR_RING_MINWIDTH = 60;
    public static int CONTOUR_MAT_PRINTOUT_NUM = 2;
    public static double CONTOUR_DILATION_KSIZE = 2.0;


    /*



     */
    public static int QUALIFY_MIN_OUT = 9;
    public static int QUALIFY_MIN_IN = 15;

    /*

     GENERAL CONSTANTS

     */
    public static final String TFOD_MODEL_ASSET_EXT     = "/sdcard/FIRST/vision/UltimateGoal.tflite";
    public static final String TFOD_FIRST_ELEMENT = "Quad";
    public static final String TFOD_SECOND_ELEMENT = "Single";

    public static final ImageResolution RESOLUTION = ImageResolution.R_960x720;

}
