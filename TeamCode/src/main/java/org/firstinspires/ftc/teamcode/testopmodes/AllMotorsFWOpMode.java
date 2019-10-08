package org.firstinspires.ftc.teamcode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.functions.ExtDirMusicPlayer;
import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;
@TeleOp(name="AllMotorsFWTest", group="Test")
public class AllMotorsFWOpMode extends OpMode {
    BasicRobot robot;
    ExtDirMusicPlayer player;
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
        } else robot.holonomic.stop();
        if (gamepad1.y) {
            player.play(ExtMusicFile.UNITY);
        } else if (gamepad1.x) player.pause();
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
