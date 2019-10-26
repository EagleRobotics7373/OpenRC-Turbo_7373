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
    private static final double TICKS_PER_REVOLUTION = 1688.06;
    private static final double TICKS_PER_INCH /* 134.4*/;
    private static final double DIAGONAL_BETWEEN_WHEELS = 23;

    private static final double ANGLE_LEFT_FRONT = 315;
    private static final double ANGLE_LEFT_REAR = 45;
    private static final double ANGLE_RIGHT_REAR = 135;
    private static final double ANGLE_RIGHT_FRONT = 225;

    private static final int TARGET_POSITION_TOLERANCE = 30;

    static {
        WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER * Math.PI);
        TICKS_PER_INCH = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;
    }

    /**
     * Creates instance with four motors and specified chassis type
     * @param frontLeftMotor front left motor
     * @param backLeftMotor back left motor
     * @param frontRightMotor front right motor
     * @param backRightMotor back right motor
     * @param chassis robot chassis type
     */
    public Holonomic(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, Chassis chassis) {
        super.frontLeftMotor = frontLeftMotor;
        super.frontRightMotor = frontRightMotor;
        super.backLeftMotor = backLeftMotor;
        super.backRightMotor = backRightMotor;
        super.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        super.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        super.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        super.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx)frontLeftMotor).setTargetPositionTolerance(TARGET_POSITION_TOLERANCE);
        ((DcMotorEx)backLeftMotor).setTargetPositionTolerance(TARGET_POSITION_TOLERANCE);
        ((DcMotorEx)frontRightMotor).setTargetPositionTolerance(TARGET_POSITION_TOLERANCE);
        ((DcMotorEx)backRightMotor).setTargetPositionTolerance(TARGET_POSITION_TOLERANCE);
        //        if (chassis == Chassis.SSGOBILDA) {
//            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        }
    }

    /**
     * Creates instance with four motors and sets chassis type to {@link Chassis#SSGOBILDA}
     * @param frontLeftMotor front left motor
     * @param backLeftMotor back left motor
     * @param frontRightMotor front right motor
     * @param backRightMotor back right motor
     */
    public Holonomic(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor) {
        this(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, Chassis.SSGOBILDA);
    }

    /**
     * Uses constants in cartesian directions + rotation to set motor power
     * Best for single-direction motion as this can sum the input powers higher than the allowed power range\
     *
     * @param x left-to-right direction
     * @param y back-to-front direction
     * @param z rotation power
     */
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


    }

    /**
     * Sets runMode for all motors to {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER}
     * Sends input powers to {@link Holonomic#run(double, double, double)}
     *
     * @see Holonomic#runWithoutEncoderVectored(double, double, double, double) for mathematically correct movement
     *
     * @param x left-to-right direction
     * @param y back-to-front direction
     * @param z rotation power
     */
    public void runWithoutEncoder(double x, double y, double z) {
        setMotorsMode(RUN_WITHOUT_ENCODER);
        run(x, y, z);
    }

    /**
     * Converts proportional constants in cartesian directions to 45-degree translated powers for optimal driving
     * Sets motor powers and runMode for all motors to {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER}
     * @param x proportional constant representing 45-degree translation of x-axis
     *               (runs from back left to front right)
     * @param y proportional constant representing 45-degree translation of y-axis
     *               (runs from back right to front left)
     * @param z value added to motor powers to create rotation
     * @param offsetTheta increases or decreases angle translation
     *                    (could be used for field-oriented drive with IMU)
     */
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
        runWithoutEncoderPrime(xPrime, yPrime, z);
    }

    /**
     * Uses 45-degree translations of the cartesian system to set motor powers
     * Sets runMode for all motors to {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER}
     * @param xPrime proportional constant representing 45-degree translation of x-axis
     *               (runs from back left to front right)
     * @param yPrime proportional constant representing 45-degree translation of y-axis
     *               (runs from back right to front left)
     * @param z value added to motor powers to create rotation
     */
    public void runWithoutEncoderPrime(double xPrime, double yPrime, double z) {
        setMotorsMode(RUN_WITHOUT_ENCODER);
        frontLeftMotor.setPower(xPrime + z);
        backLeftMotor.setPower(yPrime + z);
        backRightMotor.setPower(-xPrime + z);
        frontRightMotor.setPower(-yPrime + z);
    }

    /**
     * Uses cartesian directions to set encoder targets and motor power for strafing
     * @param xTarget left-to-right distance (inches) for the robot to travel
     * @param yTarget back-to-front distance (inches) for the robot to travel
     * @param inputPower maximum power that motors will operate at
     */
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
        frontLeftMotor.setMode(STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(STOP_AND_RESET_ENCODER);

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
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() +(int)(LFDistanceIN * TICKS_PER_INCH));
        backLeftMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + (int)(LRDistanceIN * TICKS_PER_INCH));
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + (int)(RRDistanceIN * TICKS_PER_INCH));
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + (int)(RFDistanceIN * TICKS_PER_INCH));


        // set motors mode
        frontLeftMotor.setMode(RUN_TO_POSITION);
        frontRightMotor.setMode(RUN_TO_POSITION);
        backLeftMotor.setMode(RUN_TO_POSITION);
        backRightMotor.setMode(RUN_TO_POSITION);

        // program motor power targets
        frontLeftMotor.setPower(LFPower);
        backLeftMotor.setPower(LRPower);
        backRightMotor.setPower(RRPower);
        frontRightMotor.setPower(RFPower);
    }

    /**
     * Turns the robot using encoder measurements based on a degree parameter
     * @param degrees angle in degrees for the robot to turn
     *                positive angle = clockwise
     *                negative angle = counter-clockwise
     * @param power the maximum power that motors will operate at
     */
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

    /**
     * Provides information as to whether motors are still attempting to reach encoder targets
     * @return true if all four motors are busy
     */
    public boolean motorsAreBusy() {
        int numBusy = 0;
        if (frontLeftMotor.isBusy()) numBusy++;
        if (frontLeftMotor.isBusy()) numBusy++;
        if (frontLeftMotor.isBusy()) numBusy++;
        if (frontLeftMotor.isBusy()) numBusy++;
        return (numBusy > 1);
    }

    /**
     * Sets all drivetrain motors to the same DcMotor.RunMode
     * @param runMode the RunMode to be set
     */
    public void setMotorsMode(DcMotor.RunMode runMode) {
        frontLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
    }

    /**
     * Stops all motors
     */
    @Override
    public void stop() {
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        super.stop();
    }
}