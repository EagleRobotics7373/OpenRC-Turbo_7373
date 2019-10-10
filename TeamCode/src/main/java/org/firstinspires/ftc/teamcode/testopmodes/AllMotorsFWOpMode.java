package org.firstinspires.ftc.teamcode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.library.functions.ExtDirMusicPlayer;
import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;
@TeleOp(name="AllMotorsFWTest", group="Test")
public class AllMotorsFWOpMode extends OpMode {
    BasicRobot robot;
    ExtDirMusicPlayer player;
    boolean fw = true;

    @Override
    public void init() {
        robot = new BasicRobot(hardwareMap);
        player = new ExtDirMusicPlayer();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.frontLeftMotor.setPower(1);
            robot.frontRightMotor.setPower(1);
            robot.backLeftMotor.setPower(1);
            robot.backRightMotor.setPower(1);
            telemetry.addData("Power", 1);
        } else if (gamepad1.b) {
            robot.frontLeftMotor.setPower(-1);
            robot.frontRightMotor.setPower(-1);
            robot.backLeftMotor.setPower(-1);
            robot.backRightMotor.setPower(-1);
            telemetry.addData("Power", -1);
        } else if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            robot.frontLeftMotor.setPower(gamepad1.left_stick_y);
            robot.frontRightMotor.setPower(gamepad1.left_stick_y);
            robot.backLeftMotor.setPower(gamepad1.left_stick_y);
            robot.backRightMotor.setPower(gamepad1.left_stick_y);
            telemetry.addData("Power", gamepad1.left_stick_y);
        } else {
            robot.holonomic.stop();
            telemetry.addData("Power", 0);
        }



        if (gamepad1.y) {
            player.play(ExtMusicFile.UNITY);
        } else if (gamepad1.x) player.pause();
        if (gamepad1.dpad_up) {
            robot.frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            fw = true;
        }
        else if (gamepad1.dpad_down) {
            robot.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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


    /*
    We would greatly appreciate your reviewal of our business plan and any
    level of sponsorship or support you would be willing to provide us.
     */
}
