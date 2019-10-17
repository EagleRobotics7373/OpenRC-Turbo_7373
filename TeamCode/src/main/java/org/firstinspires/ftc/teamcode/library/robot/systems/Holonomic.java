package org.firstinspires.ftc.teamcode.library.robot.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.library.functions.MathOperations;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

public class Holonomic extends Drivetrain {

    private static final double WHEEL_DIAMETER = 4;
    private static final double WHEEL_CIRCUMFERENCE;
    private static final double TICKS_PER_REVOLUTION = 537.6;
    private static final double TICKS_PER_INCH;
    private static final double DIAGONAL_BETWEEN_WHEELS = Math.sqrt(2) * 14.5;

    private static final double ANGLE_LEFT_FRONT = 315;
    private static final double ANGLE_LEFT_REAR = 45;
    private static final double ANGLE_RIGHT_REAR = 135;
    private static final double ANGLE_RIGHT_FRONT = 225;

    private static final int TARGET_POSITION_TOLERANCE = 15;

    static {
        WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER * Math.PI);
        TICKS_PER_INCH = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;
    }

    public Holonomic(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, Chassis chassis) {
        super.frontLeftMotor = frontLeftMotor;
        super.frontRightMotor = frontRightMotor;
        super.backLeftMotor = backLeftMotor;
        super.backRightMotor = backRightMotor;

        if (chassis == Chassis.SSGOBILDA) {
            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public Holonomic(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor) {
        this(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, Chassis.SSGOBILDA);
    }

    private void run(double x, double y, double z) {
        x = MathOperations.rangeClip(x, -1, 1);
        y = MathOperations.rangeClip(y, -1, 1);
        z = MathOperations.rangeClip(z, -1, 1);

        double leftFrontPower = x + y + z;
        double leftRearPower = -x + y + z;
        double rightFrontPower = x - y + z;
        double rightRearPower = -x - y + z;
        frontLeftMotor.setPower(leftFrontPower);
        backLeftMotor.setPower(leftRearPower);
        frontRightMotor.setPower(rightFrontPower);
        backRightMotor.setPower(rightRearPower);

        ((DcMotorEx)frontLeftMotor).setTargetPositionTolerance(TARGET_POSITION_TOLERANCE);
        ((DcMotorEx)backLeftMotor).setTargetPositionTolerance(TARGET_POSITION_TOLERANCE);
        ((DcMotorEx)frontRightMotor).setTargetPositionTolerance(TARGET_POSITION_TOLERANCE);
        ((DcMotorEx)backRightMotor).setTargetPositionTolerance(TARGET_POSITION_TOLERANCE);
    }

    public void runWithoutEncoderVectored(double x, double y, double z, double offsetTheta) {

        double theta;
        double axisConversionAngle = Math.PI/4;
        double xPrime;
        double yPrime;

        // calculate r
        double r = Math.sqrt(Math.pow(x,2)+ Math.pow(y,2));
        // calculate theta
        if (x == 0) x = 0.00001;
        theta = Math.atan(y / x);
        if (x < 0) theta += Math.PI;
        theta += offsetTheta;
        // calculate x and y prime
        xPrime = r * Math.cos(theta - axisConversionAngle);
        yPrime = r * Math.sin(theta - axisConversionAngle);

        // set motors mode
        setMotorsMode(RUN_WITHOUT_ENCODER);

        // set motor powers
        frontLeftMotor.setPower(xPrime+z);
        backLeftMotor.setPower(yPrime+z);
        backRightMotor.setPower(-xPrime+z);
        frontRightMotor.setPower(-yPrime+z);
    }

    public void runWithoutEncoder(double x, double y, double z) {
        setMotorsMode(RUN_WITHOUT_ENCODER);
        run(x, y, z);
    }

    public void runWithoutEncoderPrime(double xPrime, double yPrime, double z) {
        setMotorsMode(RUN_WITHOUT_ENCODER);
        frontLeftMotor.setPower(xPrime + z);
        backLeftMotor.setPower(yPrime + z);
        backRightMotor.setPower(-xPrime + z);
        frontRightMotor.setPower(-yPrime + z);
    }


    public void runUsingEncoder(double xTarget, double yTarget, double inputPower) {
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
        setMotorsMode(STOP_AND_RESET_ENCODER);

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
        frontLeftMotor.setTargetPosition((int)(LFDistanceIN * TICKS_PER_INCH));
        backLeftMotor.setTargetPosition((int)(LRDistanceIN * TICKS_PER_INCH));
        backRightMotor.setTargetPosition((int)(RRDistanceIN * TICKS_PER_INCH));
        frontRightMotor.setTargetPosition((int)(RFDistanceIN * TICKS_PER_INCH));

        // program motor power targets
        frontLeftMotor.setPower(-LFPower);
        backLeftMotor.setPower(-LRPower);
        backRightMotor.setPower(-RRPower);
        frontRightMotor.setPower(-RFPower);

        // set motors mode
        setMotorsMode(RUN_TO_POSITION);
    }

    public void turnUsingEncoder(double degrees, double power) {
        int targetPosition = (int) (degrees * TICKS_PER_REVOLUTION * (DIAGONAL_BETWEEN_WHEELS / (360 * WHEEL_DIAMETER)));
        setMotorsMode(STOP_AND_RESET_ENCODER);
        frontLeftMotor.setTargetPosition(targetPosition);
        frontRightMotor.setTargetPosition(targetPosition);
        backLeftMotor.setTargetPosition(targetPosition);
        backRightMotor.setTargetPosition(targetPosition);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        setMotorsMode(RUN_TO_POSITION);
    }

    public boolean motorsAreBusy() {
        if (frontLeftMotor.isBusy() & frontRightMotor.isBusy() & backLeftMotor.isBusy() & backRightMotor.isBusy())
            return true;
        else return false;
    }


    public void setMotorsMode(DcMotor.RunMode runMode) {
        frontLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
    }

    @Override
    public void stop() {
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        super.stop();
    }
}
