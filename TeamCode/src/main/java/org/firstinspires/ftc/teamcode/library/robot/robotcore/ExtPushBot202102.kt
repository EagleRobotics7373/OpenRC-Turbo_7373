package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.library.functions.roadrunnersupport.Encoder
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.OdometryModule
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.*
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsPushbot
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsRingPlace
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.OdometryConstantsPushbot

class ExtPushBot202102(_hardwareMap: HardwareMap) : BaseRobot(_hardwareMap) {

    init {
        println("ExtRingPlaceBot being constructed!")
        RobotConstantsAccessor.load(
                DriveConstantsPushbot::class.java,
                OdometryConstantsPushbot::class.java
        )


    }

    @JvmField val odometryLeft           : DcMotorEx              = hwInit("odometryLeft")
    @JvmField val odometryRight          : DcMotorEx              = hwInit("odometryRight")
    @JvmField val odometryRear           : DcMotorEx              = hwInit("odometryRear")

    // IMU Controller variables - allows for easier access to get heading
    @JvmField val imuControllerC          : IMUController         = IMUController(hardwareMap = hardwareMap, id = 'C')
    @JvmField val imuControllerE          : IMUController         = IMUController(hardwareMap = hardwareMap, id = 'E')

    // Odometry module variables - these will be set once we determine plug-in locations on REV Hubs
    override val leftOdometryModule: Encoder  = Encoder(odometryLeft, 8192, 38.5)
    override val rightOdometryModule: Encoder = Encoder(odometryRight,8192, 38.5)
    override val rearOdometryModule: Encoder  = Encoder(odometryRear, 8192, 38.5)

    // RoadRunner holonomic drivetrain controller and other multi-component systems
     override val holonomicRR             : HolonomicRR           = HolonomicRR(imuControllerC,
                                                                                 frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor,
                                                                                 ThreeWheelOdometryLocalizer(leftOdometryModule, rightOdometryModule, rearOdometryModule))

    init {
        super.frontLeftMotor.direction = DcMotorSimple.Direction.FORWARD
        super.backLeftMotor.direction = DcMotorSimple.Direction.FORWARD
        super.backRightMotor.direction = DcMotorSimple.Direction.FORWARD
        super.frontRightMotor.direction = DcMotorSimple.Direction.FORWARD
    }

}

