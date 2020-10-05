package org.firstinspires.ftc.teamcode.library.vision.ultimategoal;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.teamcode.library.vision.base.ImageResolution;

@Config
public class UltimateGoalVisionConstants {
    public static int BOTTOM_RING_CENTER_X = 375;
    public static int BOTTOM_RING_CENTER_Y = 315;

    public static int RING_WIDTH = 229;
    public static int RING_HEIGHT = 45;
    public static int RING_SPACING = 5;

    public static int SEARCH_SKIP = 10;
    public static RingPixelAnalysisPipeline.SearchType SEARCH_TYPE =
            RingPixelAnalysisPipeline.SearchType.AVERAGE;

    public static final String TFOD_MODEL_ASSET_EXT     = "/sdcard/FIRST/vision/UltimateGoal.tflite";
    public static final String TFOD_FIRST_ELEMENT = "Quad";
    public static final String TFOD_SECOND_ELEMENT = "Single";

    public static ImageResolution RESOLUTION = ImageResolution.R_960x720;

}
