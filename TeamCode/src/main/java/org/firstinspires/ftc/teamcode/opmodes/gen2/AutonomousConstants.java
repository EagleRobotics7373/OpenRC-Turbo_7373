package org.firstinspires.ftc.teamcode.opmodes.gen2;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile;
import org.firstinspires.ftc.teamcode.library.functions.FieldSide;
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsTunedMisumi;

@Config
public class AutonomousConstants {
    public static AllianceColor _allianceColor = AllianceColor.RED;
    public static FieldSide _startingPosition = FieldSide.LOADING_ZONE;
    public static AutonomousType _autonomousType = AutonomousType.MULTISTONE;
    public static Class<?> _driveClass = DriveConstantsTunedMisumi.class;
    public static ExtMusicFile _musicFile = ExtMusicFile.NONE;
    public static boolean _parkOnly = false;
    public static int _delayBeforeParking = 0;
    public static boolean _foundationSwivel = false;
    public static boolean _doFoundationPull = true;
    public static boolean _doParkAtEnd = true;
    public static boolean _liftLowerOnly = false;
    public static int _numStonesToMove = 3;

    // other things
    public static double RR_NEXT_TO_STONE_Y = 35.75;
    public static double RR_AGAINST_BRIDGE_Y = 42.5;
    public static double RR_AGAINST_FOUNDATION_Y = 32.0;
    public static double RR_PAST_BRIDGE_X = 4.0;
    public static double RR_RED_STONE_OFFSET = 3.0;
    public static int INTAKE_DROP_POSITION = -205;

    // stones
//    public static int NUM_STONES = 2;
    public static int STONE_0 = 1;
    public static int STONE_1 = 5;
    public static int STONE_2 = 4;
    public static int STONE_3 = 3;

}
