package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor;
import org.firstinspires.ftc.teamcode.library.functions.ExtDirMusicPlayer;
import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile;
import org.firstinspires.ftc.teamcode.library.functions.FieldSide;
import org.firstinspires.ftc.teamcode.library.functions.MathExtensionsKt;
import org.firstinspires.ftc.teamcode.library.functions.Point3D;
import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController;
import org.firstinspires.ftc.teamcode.library.vision.skystone.VisionInitializer;
import org.firstinspires.ftc.teamcode.library.vision.skystone.VuforiaController;

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
        imuController = new IMUController(hardwareMap, AxesOrder.ZYX);
        AutoMenuControllerIterative menuController = new AutoMenuControllerIterative(telemetry);


        /*
                Operate telemetry menu
         */
        while (!isStarted() && !isStopRequested()) {
           if (gamepad1.dpad_up) {
               menuController.menu.previousItem();
               while (gamepad1.dpad_up && !isStopRequested());
           } else if (gamepad1.dpad_down) {
               menuController.menu.nextItem();
               while (gamepad1.dpad_down && !isStopRequested());
           } else if (gamepad1.dpad_left) {
                menuController.menu.iterateBackward();
                while (gamepad1.dpad_left && !isStopRequested());
           } else if (gamepad1.dpad_right) {
               menuController.menu.iterateForward();
               while (gamepad1.dpad_right && !isStopRequested());
           }
        }
        waitForStart();
        if (!isStopRequested()) {
        /*
                RoboSpotify
         */
            ExtDirMusicPlayer player = new ExtDirMusicPlayer(menuController.getMusicFile(), true);
            player.play();


        /*
                Robot Actions
         */


            if (menuController.getStartingPosition()== FieldSide.WAFFLE_SIDE) {
                if (menuController.getAllianceColor()== AllianceColor.RED) {
                    // Drive forward to clear the wall
    //                drive(0, 5, 0.7);
    //                sleep(500);
    //                // Rotate 180 degrees using IMU PI controller
    //                imuPIRotate(180);

                    // Find distance away from the wall
                    double distWall = 42 - robot.distanceSensor_side.getDistance(DistanceUnit.INCH);

                    // Drive to the foundation
                    drive(0, -29, 0.2);
                    sleep(250);

                    // Deploy the foundation grabber, grabbing the foundation
                    robot.foundationGrabbers.lock();
                    sleep(2000);

                    // Drive back to the wall
                    drive(0, 32, 0.7);

                    // Release the foundation grabbers
                    robot.foundationGrabbers.unlock();
                    sleep(500);

                    // Drive forward to get clearance from the foundation
                    drive(0, 2, 0.7);

                    // Drive toward the alliance bridge to start moving around the foundation
                    drive(distWall, 0, 0.7);
                    // Drive parallel to the bridges to move to the other side of the foundation
                    drive(0, -18, 0.7);
                    drive(-20, 0, 0.7);

                    // Park under the bridge
                    drive(23, 0, 0.2);
                    if (menuController.getParkNearDS()) timeDrive(0,0.8, 0, 1000);
                    else timeDrive(0, -0.5, 0, 500);
                } else { // Blue side waffle
                    // Drive forward to clear the wall
    //                drive(0, 5, 0.4);
    //                sleep(500);
                    // Find distance away from the wall
    //                double distWall = 44 - robot.distanceSensor_side.getDistance(DistanceUnit.INCH);

                    // Drive to the foundation
                    drive(0, -29, 0.2);
                    sleep(250);

                    // Deploy the foundation grabber, grabbing the foundation
                    robot.foundationGrabbers.lock();
                    sleep(2000);

                    // Drive back to the wall
                    drive(0, 36, 0.2);

                    // Release the foundation grabbers
                    robot.foundationGrabbers.unlock();
                    sleep(500);

                    // Drive toward the alliance bridge to start moving around the foundation
                    drive(-32, 0, 0.7);
                    // Drive parallel to the bridges to move to the other side of the foundation
                    drive(0, -15.5, 0.7);
                    //push foundation
                    drive(20, 0, 0.5);
                    sleep(1000);
                    // drive back
                    drive(-26, 0, 0.7);
                    if(menuController.getParkNearDS()) drive(0, 24,0.2);
                    else {
                        timeDrive(0, -0.4, 0, 500);
                        sleep(500);
                        robot.holonomic.stop();
                    }
                }
            } else { // FIELD POSITION IS LOADING ZONE!!!
                VuforiaLocalizer vuforia = VisionInitializer.createVuforia(VisionInitializer.CameraType.PHONE_REAR, hardwareMap);
                VuforiaController vuforiaController = new VuforiaController(vuforia, telemetry);
                vuforiaController.activate();

                // Drive closer to the stone to see it more reliably
                drive(15, 0, .2);
                sleep(1000);
                Point3D vuforiaTargetPoint = vuforiaController.analyzeVuforiaResult();
                for (int i = 0; i < 5 & vuforiaTargetPoint==null; i++) {
                    vuforiaTargetPoint = vuforiaController.analyzeVuforiaResult();
                    sleep(500);
                }

                Position skystonePosition = Position.RIGHT;

                if (vuforiaTargetPoint != null) {
                    telemetry.addData("Skystone y", vuforiaTargetPoint.y);
                    if (vuforiaTargetPoint.y < 1.0) skystonePosition = Position.LEFT;
                    else skystonePosition = Position.CENTER;
                }

                telemetry.addData("Skystone position", skystonePosition);
                telemetry.update();
                double driveToSkystoneDist;
                if (menuController.getAllianceColor()==AllianceColor.RED) {
                    switch (skystonePosition) {
                        case CENTER:
                            driveToSkystoneDist = 2.0;
                            break;
                        case RIGHT:
                            driveToSkystoneDist = -7.0;
                            break;
                        default:
                            driveToSkystoneDist = 10.5;
                            break;
                    }
                } else {
                    switch (skystonePosition) {
                        case CENTER:
                            driveToSkystoneDist = 2.75;
                            break;
                        case RIGHT:
                            driveToSkystoneDist = -6.0;
                            break;
                        default:
                            driveToSkystoneDist = 10.5;
                            break;
                    }
                }

                // Drive forwards or back to align with
                drive(0, driveToSkystoneDist, 0.2);
                sleep(2000);
                // Rotate to face Skystone
                imuPIRotate(-88.0);

                // PI controller for lifting arm
                doArmLift();

                /*

                IMPORTANT!!! (AKA REALLY IMPORTANT!!!)
                Check location of these blocks in case below y value needs to be increased/decreased!!!

                 */
                double targetValue = 18.7;
                double currentValue;
                double P = 0.03;

                while (!MathExtensionsKt.withinRange(currentValue = robot.distanceSensor_front.getDistance(DistanceUnit.CM), targetValue, 0.3)) {
                    robot.holonomic.runWithoutEncoder(0,MathExtensionsKt.upperLimit(P * (currentValue-targetValue), 0.5),0);
                    telemetry.addData("Target", targetValue);
                    telemetry.addData("Current", currentValue);
                    telemetry.update();
                }

//                drive(0, 16, 0.2);
                sleep(750);
                robot.intakeBlockManipulator.setPower(1);

                robot.intakePivotMotor.setPower(0.0);
                robot.holonomic.runWithoutEncoder(0,-0.12,0);
                sleep(500);
                robot.holonomic.stop();
                sleep(1500);

                doArmLift();

                drive(0, -6, 0.4);

                double intoBuildingZoneDist = 0.0;
                if (menuController.getAllianceColor() == AllianceColor.RED) {
                    switch (skystonePosition) {
                        case LEFT: intoBuildingZoneDist+=8;
                        case CENTER: intoBuildingZoneDist+=8;
                        case RIGHT: intoBuildingZoneDist+=40;
                    }
                } else {
                    switch (skystonePosition) {
                        case RIGHT: intoBuildingZoneDist-=8;
                        case CENTER: intoBuildingZoneDist-=8;
                        case LEFT: intoBuildingZoneDist-=40;

                    }
                }
                drive(intoBuildingZoneDist, 0, 0.3);

                robot.intakeBlockManipulator.setPower(-1.0);

                if (menuController.getAllianceColor() == AllianceColor.RED) drive(-20,0,0.2);
                else drive(20, 0, 0.2);

                if (menuController.getParkNearDS()) timeDrive(0, -0.7, 0, 750);
                else                                timeDrive(0, 0.4, 0, 750);
            }


        /*
                End of OpMode - close resources
         */
            player.stop();
        }
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
        double originalRuntime = getRuntime();
        while (currentValue != targetValue && opModeIsActive() && (getRuntime()-originalRuntime)<3) {
            currentValue = MathExtensionsKt.toDegrees(imuController.getHeading());
            telemetry.addData("Current value", currentValue);
            telemetry.addData("Target value", targetValue);

            if (currentValue < 0) {
                et = -(Math.abs(targetValue) - Math.abs(currentValue));
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
            telemetry.addData("et", et);
            telemetry.addData("propPw", proportionPower);
            telemetry.addData("intPw", integralPower);
            telemetry.addData("errorsum", errorSum);
            telemetry.addData("Power", power);
            robot.holonomic.runWithoutEncoder(0,0,power);
            telemetry.update();
        }
        robot.holonomic.stop();
    }

    private void doArmLift() {
        double currentValue = 0;
        double targetValue = 1.257;
        double PPower = 4.0;
        double originalRuntime = getRuntime();
        while (currentValue != targetValue && opModeIsActive() && (getRuntime()-originalRuntime)<2) {
            currentValue = robot.intakePivotPotentiometer.getVoltage();
            robot.intakePivotMotor.setPower(-PPower * (targetValue-robot.intakePivotPotentiometer.getVoltage()));
        }
        robot.intakePivotMotor.setPower(0.05);
    }

    private void timeDrive(double x, double y, double z, long timeMs) {
        robot.holonomic.runWithoutEncoder(x, y, z);
        sleep(timeMs);
        robot.holonomic.stop();
    }

    private void drive(double x, double y, double power) {
        robot.holonomic.runUsingEncoder(x, y, power);
        double originalRuntime = getRuntime();
        while (opModeIsActive() && robot.holonomic.motorsAreBusy() && getRuntime()-originalRuntime<3) {

        }
    }

}
