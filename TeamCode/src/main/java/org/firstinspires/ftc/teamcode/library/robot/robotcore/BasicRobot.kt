package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.library.robot.systems.FoundationGrabbers
import org.firstinspires.ftc.teamcode.library.robot.systems.Holonomic

class BasicRobot(private val hardwareMap: HardwareMap) {
    // Drivetrain Variables
     @JvmField val frontLeftMotor          : DcMotor               = hwInit("frontLeftMotor")
     @JvmField val backLeftMotor           : DcMotor               = hwInit("backLeftMotor")
     @JvmField val frontRightMotor         : DcMotor               = hwInit("frontRightMotor")
     @JvmField val backRightMotor          : DcMotor               = hwInit("backRightMotor")

     @JvmField val intakeBlockManipulator  : DcMotor               = hwInit("intakeBlockManipulator")
     @JvmField val intakePivotMotor        : DcMotor               = hwInit("intakePivotMotor")


    // Servo Variables
     private   val foundationServo         : Servo                 = hwInit("foundationServo")
//     private   val backRightFoundationServo: Servo                 = hwInit("backRightFoundationServo")

    // IMU Variables
//     @JvmField val imuA                    : BNO055IMU             = hwInit("imuA")
//     @JvmField val imuB                    : BNO055IMU             = hwInit("imuB")

    // Analog Input Variables
     @JvmField val intakePivotPotentiometer: AnalogInput           = hwInit("potentiometer")

    // Color/Distance Sensor Variables
     @JvmField val intakeBlockCSensor      : ColorSensor           = hwInit("intakeBlockSensor")
     @JvmField val intakeBlockDSensor      : DistanceSensor        = hwInit("intakeBlockSensor")

    // Robot Systems Variables
     @JvmField val holonomic               : Holonomic              = Holonomic(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor)
     @JvmField val foundationGrabbers      : FoundationGrabbers     = FoundationGrabbers(foundationServo)
    //     @JvmField val blinkin                 : RevBlinkinLedDriver   = hwInit("blinkin")
    private inline fun <reified T> hwInit(name:String): T = hardwareMap.get(T::class.java, name)
}
