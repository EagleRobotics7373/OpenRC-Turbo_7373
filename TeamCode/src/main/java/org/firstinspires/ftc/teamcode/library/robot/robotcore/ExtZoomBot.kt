package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.OdometryModule
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.*
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsRingPlace
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.OdometryConstants
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.BlinkinController
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.RingTapper
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.WobbleGrabber

open class ExtZoomBot(_hardwareMap: HardwareMap) : BaseRobot(_hardwareMap) {

    init {
        println("ExtZoomBot being constructed!")
        RobotConstantsAccessor.load(
                DriveConstantsRingPlace::class.java,
                OdometryConstants::class.java
        )
    }



    @JvmField val intakeStage1           : DcMotorEx              = hwInit("intakeStage1")
    @JvmField val intakeStage2           : DcMotorEx              = hwInit("intakeStage2")
    @JvmField val zoomWheel              : DcMotorEx              = hwInit("zoomWheel")

    init { intakeStage2.direction = DcMotorSimple.Direction.REVERSE }
    init { zoomWheel.direction    = DcMotorSimple.Direction.REVERSE }
    init { zoomWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ExtZoomBotConstants.VELOCITY_PID)}

    @JvmField val ringLoadServo          : Servo                  = hwInit("ringLoadServo")
    @JvmField val ringTapperServo        : Servo                  = hwInit("ringTapperServo")

    @JvmField val wobblePivotServo       : Servo                  = hwInit("wobblePivotServo")
    @JvmField val wobbleGrabServo        : Servo                  = hwInit("wobbleGrabServo")

    @JvmField val odometryLeft           : DcMotorEx              = this.intakeStage2
    @JvmField val odometryRear           : DcMotorEx              = hwInit("odometryRear")


    // Odometry module variables - these will be set once we determine plug-in locations on REV Hubs
    override val leftOdometryModule: OdometryModule?  = null
    override val rightOdometryModule: OdometryModule? = null
    override val rearOdometryModule: OdometryModule?  = null

    // LED module - can be uncommented when installed on robot
    @JvmField val blinkin                 : RevBlinkinLedDriver = hwInit("blinkin")
    @JvmField val blinkinController       : BlinkinController = BlinkinController(blinkin, 1000)

    // IMU Controller variables - allows for easier access to get heading
    @JvmField val imuControllerC          : IMUController         = IMUController(hardwareMap = hardwareMap, id = 'C')
    @JvmField val imuControllerE          : IMUController         = IMUController(hardwareMap = hardwareMap, id = 'E')

    // RoadRunner holonomic drivetrain controller and other multi-component systems
     override val holonomicRR             : HolonomicRR           = HolonomicRR(imuControllerC,
                                                                                 frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor,
                                                                                 TwoWheelOdometryLocalizer(odometryLeft, odometryRear, imuControllerC))
    @JvmField val ringTapper           : RingTapper = RingTapper(ringTapperServo)
    @JvmField val wobbleGrabber           : WobbleGrabber = WobbleGrabber(wobblePivotServo, wobbleGrabServo)
}
