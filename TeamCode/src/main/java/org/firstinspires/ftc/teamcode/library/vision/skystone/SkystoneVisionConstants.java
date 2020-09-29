package org.firstinspires.ftc.teamcode.library.vision.skystone;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Point;

@Config
public class SkystoneVisionConstants {
    //TensorFlow Constants
    public static final String  TFOD_LABEL_SKYSTONE      = "Skystone";
    public static final String  TFOD_LABEL_STONE         = "Stone";
    public static final String  TFOD_MODEL_ASSET_LOCAL   = "Skystone.tflite";
    public static final String  TFOD_MODEL_ASSET_EXT     = "/sdcard/FIRST/Skystone.tflite";

    //OpenCV Hue Stats Pipeline Constants
    public static       int     HS_STONE_WIDTH           = 90;
    public static       int     HS_STONE_GAP             = /*40*/ 85;
    public static       Point   HS_FIRST_STONE_PT        = new Point(/*270*/ 250, 273);
}
