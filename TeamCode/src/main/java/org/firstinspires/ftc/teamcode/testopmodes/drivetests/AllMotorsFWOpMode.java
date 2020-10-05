package org.firstinspires.ftc.teamcode.testopmodes.drivetests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.library.functions.ExtDirMusicPlayer;
import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BaseRobot;
import static org.firstinspires.ftc.teamcode.library.robot.robotcore.RobotProvider.providePresetRobot;

@Config
@TeleOp(name="AllMotorsFWTest", group="Test")
public class AllMotorsFWOpMode extends OpMode {
    BaseRobot robot;
    ExtDirMusicPlayer player;
    boolean fw = true;

    public static double defaultSpeed = 0.2;
    
    @Override
    public void init() {
        robot = providePresetRobot(hardwareMap);
        player = new ExtDirMusicPlayer(ExtMusicFile.MEGALOUNITY);
        player.play();
    }
    
    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.frontLeftMotor.setPower(defaultSpeed);
            robot.frontRightMotor.setPower(defaultSpeed);
            robot.backLeftMotor.setPower(defaultSpeed);
            robot.backRightMotor.setPower(defaultSpeed);
            telemetry.addData("Power", 1);
        } else if (gamepad1.b) {
            robot.frontLeftMotor.setPower(defaultSpeed);
            robot.frontRightMotor.setPower(defaultSpeed);
            robot.backLeftMotor.setPower(defaultSpeed);
            robot.backRightMotor.setPower(defaultSpeed);
            telemetry.addData("Power", -1);
        } else if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            robot.frontLeftMotor.setPower(gamepad1.left_stick_y);
            robot.frontRightMotor.setPower(gamepad1.left_stick_y);
            robot.backLeftMotor.setPower(gamepad1.left_stick_y);
            robot.backRightMotor.setPower(gamepad1.left_stick_y);
            telemetry.addData("Power", gamepad1.left_stick_y);
        } else {
            robot.holonomic.stop();
            if (gamepad2.dpad_up) {
                if (gamepad2.dpad_right) robot.frontRightMotor.setPower(defaultSpeed);
                if (gamepad2.dpad_left) robot.frontLeftMotor.setPower(defaultSpeed);
            }
            if (gamepad2.dpad_down) {
                if (gamepad2.dpad_right) robot.backRightMotor.setPower(defaultSpeed);
                if (gamepad2.dpad_left) robot.backLeftMotor.setPower(defaultSpeed);
            }
            telemetry.addData("Power", 0);
        }



        if (gamepad1.y) {
            player.play();
        } else if (gamepad1.x) player.pause();
        if (gamepad1.dpad_up) {
            robot.frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            fw = true;
        }
        else if (gamepad1.dpad_down) {
            robot.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            fw = false;
        }
        telemetry.addData("Direction", fw?"Forward":"Reverse");
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        player.stop();
    }
}
