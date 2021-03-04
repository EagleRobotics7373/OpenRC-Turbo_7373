package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.library.functions.roadrunnersupport.Encoder
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.OdometryModule
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.*
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsRingPlace
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.OdometryConstants
import org.firstinspires.ftc.teamcode.library.robot.systems.intakegen3.MagazineRingSensor
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
    @JvmField val intakeStage2S          : CRServo                = hwInit("intakeStage2S")
    @JvmField val ringTapThruServo       : Servo                  = hwInit("ringTapThruServo")

    @JvmField val wobblePivotServo       : Servo                  = hwInit("wobblePivotServo")
    @JvmField val wobbleGrabServo        : Servo                  = hwInit("wobbleGrabServo")

    @JvmField val deflectionServo       : Servo                   = hwInit("ringDeflectionServo")

    @JvmField val odometryLeft           : DcMotorEx              = this.intakeStage2
    @JvmField val odometryRear           : DcMotorEx              = hwInit("odometryRear")

    @JvmField val magColorSensor1        : ColorSensor            = hwInit("mgcs1")
    @JvmField val magColorSensor2        : ColorSensor            = hwInit("mgcs2")
    @JvmField val magColorSensor3        : ColorSensor            = hwInit("mgcs3")

    @JvmField val magColorSensor1D       : DistanceSensor         = hwInit("mgcs1")
    @JvmField val magColorSensor2D       : DistanceSensor         = hwInit("mgcs2")
    @JvmField val magColorSensor3D       : DistanceSensor         = hwInit("mgcs3")


    // Odometry module variables - these will be set once we determine plug-in locations on REV Hubs
    override val leftOdometryModule: Encoder?  = null
    override val rightOdometryModule: Encoder? = null
    override val rearOdometryModule: Encoder?  = null

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
    @JvmField val ringTapThru             : RingTapper = RingTapper(ringTapThruServo, 0.87, 0.55)
    @JvmField val ringTapper              : RingTapper = RingTapper(ringTapperServo, 0.6, 0.45)
    @JvmField val wobbleGrabber           : WobbleGrabber = WobbleGrabber(wobblePivotServo, wobbleGrabServo)

    @JvmField val magazineRingSensor1     : MagazineRingSensor = MagazineRingSensor(magColorSensor1, magColorSensor1D)
    @JvmField val magazineRingSensor2     : MagazineRingSensor = MagazineRingSensor(magColorSensor2, magColorSensor2D)
    @JvmField val magazineRingSensor3     : MagazineRingSensor = MagazineRingSensor(magColorSensor3, magColorSensor3D)
}
