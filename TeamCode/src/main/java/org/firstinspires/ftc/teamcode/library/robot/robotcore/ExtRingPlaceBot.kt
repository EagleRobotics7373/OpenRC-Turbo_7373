package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.OdometryModule
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.*
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsTunedMisumi
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.OdometryConstants

open class ExtRingPlaceBot(_hardwareMap: HardwareMap) : BaseRobot(_hardwareMap) {

    init {
        println("ExtRingPlaceBot being constructed!")
        RobotConstantsAccessor.load(
                DriveConstantsTunedMisumi::class.java,
                OdometryConstants::class.java
        )
    }

    // Drivetrain Variables
            // @JvmField val frontLeftMotor  : DcMotorEx FROM BaseRobot
            // @JvmField val backLeftMotor   : DcMotorEx FROM BaseRobot
            // @JvmField val frontRightMotor : DcMotorEx FROM BaseRobot
            // @JvmField val backRightMotor  : DcMotorEx FROM BaseRobot

    // Odometry module variables - these will be set once we determine plug-in locations on REV Hubs
    override val leftOdometryModule: OdometryModule?  = null
    override val rightOdometryModule: OdometryModule? = null
    override val rearOdometryModule: OdometryModule?  = null

    // LED module - can be uncommented when installed on robot
//    @JvmField val blinkin                 : RevBlinkinLedDriver   = hwInit("blinkin")

    // IMU Controller variables - allows for easier access to get heading
    @JvmField val imuControllerC          : IMUController         = IMUController(hardwareMap = hardwareMap, id = 'C')
    @JvmField val imuControllerE          : IMUController         = IMUController(hardwareMap = hardwareMap, id = 'E')

    // RoadRunner holonomic drivetrain controller
     override val holonomicRR             : HolonomicRR           = HolonomicRR(imuControllerC,
                                                                                 frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor)
}
