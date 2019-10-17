package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.library.functions.ExtDirMusicPlayer;
import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile;
import org.firstinspires.ftc.teamcode.library.functions.MathExtensionsKt;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Main")
public class Autonomous extends LinearOpMode {
    BasicRobot robot;
    IMUController imuController;
    @Override
    public void runOpMode() throws InterruptedException {

        /*
                Initialize main autonomous variables
         */
        robot = new BasicRobot(hardwareMap);
        imuController = new IMUController(hardwareMap, AxesOrder.XYZ);
        AutoParamMenu menu = new AutoParamMenu(telemetry);


        /*
                Operate telemetry menu
         */
        while (opModeIsActive() && !isStarted()) {
           if (gamepad1.dpad_up) {
               menu.menu.previousItem();
               while (gamepad1.dpad_up && opModeIsActive());
           } else if (gamepad1.dpad_down) {
               menu.menu.nextItem();
               while (gamepad1.dpad_down && opModeIsActive());
           } else if (gamepad1.dpad_left) {
                menu.menu.iterateCurrentItemBackward();
                while (gamepad1.dpad_left && opModeIsActive());
           } else if (gamepad1.dpad_right) {
               menu.menu.iterateCurrentItemForward();
               while (gamepad1.dpad_right && opModeIsActive()) ;
           }
        }
        waitForStart();
        /*
                RoboSpotify
         */
//        ExtDirMusicPlayer player = new ExtDirMusicPlayer(menu.getMusicFile());
//        player.play();


        /*
                Robot Actions
         */
        robot.holonomic.runWithoutEncoder(0,1,0);
        sleep(800);
        robot.holonomic.stop();
        robot.foundationGrabbers.lock();
        sleep(1500);
        robot.holonomic.runWithoutEncoder(0,-1,0);
        sleep(2000);
        robot.holonomic.stop();
        robot.foundationGrabbers.unlock();
        sleep(1500);
        drive(0,-3, 1);
        drive(-36, 0, 1);
        drive(0, -48, 1);
        drive(36, 0, 1);
        robot.holonomic.runWithoutEncoder(0, -1, 0);
        sleep(1000);
        robot.holonomic.stop();


        /*
                End of OpMode - close resources
         */
//        player.stop();
    }

    public void imuPIRotate(double angle) {
        double currentValue = MathExtensionsKt.toDegrees(imuController.getHeading());
        double targetValue = currentValue + angle;

        double Kp = .02; // Proportional Constant
        double Ki = .0002; // Integral Constant
        double et; // Error
        double proportionPower;
        double integralPower;
        double power;
        double errorSum = 0;

        if (targetValue >= 360) targetValue-=360;
        if (targetValue <= 360) targetValue+=360;



        while (currentValue != targetValue && opModeIsActive()) {
            currentValue = MathExtensionsKt.toDegrees(imuController.getHeading());
            if (currentValue < 0) {
                et = Math.abs(targetValue) - (360 - Math.abs(currentValue));
            } else {
                et = targetValue - currentValue;
            }


            if (Kp * et > .8) {
                proportionPower = .8;
            } else {
                proportionPower = Kp * et;
            }

            if (Math.abs(et) < 45) {
                errorSum += et;
            }

            integralPower = Ki * errorSum;

            power = -(proportionPower + integralPower);

            robot.backLeftMotor.setPower(power);
            robot.backRightMotor.setPower(power);
            robot.frontLeftMotor.setPower(power);
            robot.frontRightMotor.setPower(power);
        }
    }

    private void drive(double x, double y, double power) {
        robot.holonomic.runUsingEncoder(x, y, power);
        while (opModeIsActive() && robot.holonomic.motorsAreBusy()) {

        }
    }

}
