package org.firstinspires.ftc.teamcode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@Autonomous(name="More Basic Encoder Test", group="Test")
public class MoreBasicEncoderDriveTest extends LinearOpMode {
    BasicRobot robot;
    static final double TICKS_PER_INCH = 134.4;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BasicRobot(hardwareMap);
        drive(10, 0, 0.6);
        while (robot.holonomic.motorsAreBusy());
        robot.holonomic.stop();
        sleep(2000);
    }

    public void drive(double xTarget, double yTarget, double inputPower) {
        double r;
        double theta;
        double axisConversionAngle = Math.PI/4;
        double xPrime;
        double yPrime;
        double xPower;
        double yPower;
        double LFDistanceIN;
        double LRDistanceIN;
        double RRDistanceIN;
        double RFDistanceIN;
        double LFPower;
        double LRPower;
        double RRPower;
        double RFPower;

        // set motors mode
        robot.frontLeftMotor.setMode(STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(STOP_AND_RESET_ENCODER);


        // calculate r
        r = Math.sqrt(Math.pow(xTarget,2)+ Math.pow(yTarget,2));
        // calculate theta
        if (xTarget == 0) xTarget = 0.00001;
        theta = Math.atan(yTarget / xTarget);
        if (xTarget < 0) theta += Math.PI;
        // calculate x and y prime
        xPrime = r * Math.cos(theta - axisConversionAngle);
        yPrime = r * Math.sin(theta - axisConversionAngle);

        // calculate x and y power
        if (Math.abs(yPrime) > Math.abs(xPrime)) {
            yPower = inputPower;
            xPower = inputPower * (xPrime / yPrime);
        } else {
            xPower = inputPower;
            yPower = inputPower * (yPrime / xPrime);
        }

        // set motor distances (inches)
        LFDistanceIN = xPrime;
        LRDistanceIN = yPrime;
        RRDistanceIN = -xPrime;
        RFDistanceIN = -yPrime;

        // set motor powers
        LFPower = xPower;
        LRPower = yPower;
        RRPower = -xPower;
        RFPower = -yPower;

        // program encoder targets
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() +(int)(LFDistanceIN * TICKS_PER_INCH));
        robot.backLeftMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + (int)(LRDistanceIN * TICKS_PER_INCH));
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + (int)(RRDistanceIN * TICKS_PER_INCH));
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + (int)(RFDistanceIN * TICKS_PER_INCH));

        // program motor power targets
        robot.frontLeftMotor.setPower(LFPower);
        robot.backLeftMotor.setPower(LRPower);
        robot.backRightMotor.setPower(RRPower);
        robot.frontRightMotor.setPower(RFPower);

        telemetry.addData("flm target", robot.frontLeftMotor.getTargetPosition());
        telemetry.addData("frm target", robot.frontRightMotor.getTargetPosition());
        telemetry.addData("blm target", robot.backLeftMotor.getTargetPosition());
        telemetry.addData("brm target", robot.backRightMotor.getTargetPosition());
        telemetry.update();
        // set motors mode
        robot.frontLeftMotor.setMode(RUN_TO_POSITION);
        robot.frontRightMotor.setMode(RUN_TO_POSITION);
        robot.backLeftMotor.setMode(RUN_TO_POSITION);
        robot.backRightMotor.setMode(RUN_TO_POSITION);
    }


}
