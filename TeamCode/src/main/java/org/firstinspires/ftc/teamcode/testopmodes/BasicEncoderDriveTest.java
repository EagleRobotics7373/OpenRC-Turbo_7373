package org.firstinspires.ftc.teamcode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;

@Autonomous(name="Encoder Drive Test", group="Test")
public class BasicEncoderDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(hardwareMap);
        waitForStart();
        robot.holonomic.turnUsingEncoder(90, 0.6);
        do {
            telemetry.addData("FL target", robot.frontLeftMotor.getTargetPosition());
            telemetry.addData("FL ticks", robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("FL power", robot.frontLeftMotor.getPower());
            telemetry.addData("FR target", robot.frontRightMotor.getTargetPosition());
            telemetry.addData("FR ticks", robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("FR power", robot.frontRightMotor.getPower());
            telemetry.addData("BL target", robot.backLeftMotor.getTargetPosition());
            telemetry.addData("BL ticks", robot.backLeftMotor.getCurrentPosition());
            telemetry.addData("BL power", robot.backLeftMotor.getPower());
            telemetry.addData("BR target", robot.backRightMotor.getTargetPosition());
            telemetry.addData("BR ticks", robot.backRightMotor.getCurrentPosition());
            telemetry.addData("BR power", robot.backRightMotor.getPower());
            telemetry.update();
        } while (robot.holonomic.motorsAreBusy());
    }
}

