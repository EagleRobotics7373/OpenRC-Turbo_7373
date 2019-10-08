package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.library.robot.systems.Holonomic

class BasicRobot(private val hardwareMap: HardwareMap) {
    // Drivetrain (DcMotor) Variables
     @JvmField val frontLeftMotor          : DcMotor               = hwInit("frontLeftMotor")
     @JvmField val backLeftMotor           : DcMotor               = hwInit("backLeftMotor")
     @JvmField val frontRightMotor         : DcMotor               = hwInit("frontRightMotor")
     @JvmField val backRightMotor          : DcMotor               = hwInit("backRightMotor")

    // Servo Variables
//     @JvmField val teamMarkerServo         : Servo                 = hwInit("teamMarkerServo")


    // IMU Variables
//     @JvmField val imuA                    : BNO055IMU             = hwInit("imuA")
//     @JvmField val imuB                    : BNO055IMU             = hwInit("imuB")

    // Analog Input Variables


    // Color/Distance Sensor Variables


    // Robot Systems Variables
     @JvmField val holonomic               : Holonomic = Holonomic(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor)
//     @JvmField val blinkin                 : RevBlinkinLedDriver   = hwInit("blinkin")
    private inline fun <reified T> hwInit(name:String): T = hardwareMap.get(T::class.java, name)
}
