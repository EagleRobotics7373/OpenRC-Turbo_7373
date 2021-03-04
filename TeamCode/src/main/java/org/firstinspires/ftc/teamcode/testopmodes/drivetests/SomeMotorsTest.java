package org.firstinspires.ftc.teamcode.testopmodes.drivetests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sun.tools.javac.util.AbstractLog;

import org.firstinspires.ftc.teamcode.library.functions.ExtDirMusicPlayer;
import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BaseRobot;

import static org.firstinspires.ftc.teamcode.library.robot.robotcore.RobotProvider.providePresetRobot;

@Config
@TeleOp(name="Some Motors Test", group="Test")
public class SomeMotorsTest extends OpMode {
    BaseRobot robot;
    public static double SPEED = 0.05;
    public static boolean FRONT_LEFT_ON = true;
    public static boolean BACK_LEFT_ON = true;
    public static boolean BACK_RIGHT_ON = true;
    public static boolean FRONT_RIGHT_ON = true;


    @Override
    public void init() {
        robot = providePresetRobot(hardwareMap);
        robot.holonomic.setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    @Override
    public void loop() {
        double power;
        if (gamepad1.a) power = SPEED;
        else if (gamepad1.b) power = -SPEED;
        else power = 0;

        robot.frontLeftMotor.setPower(FRONT_LEFT_ON ? power : 0);
        robot.backLeftMotor.setPower(BACK_LEFT_ON ? power : 0);
        robot.backRightMotor.setPower(BACK_RIGHT_ON ? power : 0);
        robot.frontRightMotor.setPower(FRONT_RIGHT_ON ? power : 0);
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
