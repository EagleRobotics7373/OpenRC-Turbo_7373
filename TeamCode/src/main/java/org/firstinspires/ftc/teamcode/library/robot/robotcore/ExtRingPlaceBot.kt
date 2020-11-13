package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.OdometryModule
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.*
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsRingPlace
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsTunedMisumi
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.OdometryConstants
import org.firstinspires.ftc.teamcode.library.robot.systems.intake.FullIntakeSystem
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.RingDropper
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.WobbleGrabber

open class ExtRingPlaceBot(_hardwareMap: HardwareMap) : BaseRobot(_hardwareMap) {

    init {
        println("ExtRingPlaceBot being constructed!")
        RobotConstantsAccessor.load(
                DriveConstantsRingPlace::class.java,
                OdometryConstants::class.java
        )
    }

    @JvmField val odometryLeft           : DcMotorEx              = hwInit("odometryLeft")
    @JvmField val odometryRear           : DcMotorEx              = hwInit("odometryRear")

    @JvmField val intakeLiftMotor        : DcMotorEx              = hwInit("intakeLiftMotor")
    @JvmField val ringIntakeMotor        : DcMotorEx              = hwInit("ringIntakeMotor")
    @JvmField val ringDropServo          : Servo                  = hwInit("ringDropServo")
    @JvmField val liftPotentiometer      : AnalogInput            = hwInit("liftPotentiometer")

    @JvmField val wobblePivotServo       : Servo                  = hwInit("wobblePivotServo")
    @JvmField val wobbleGrabServo        : Servo                  = hwInit("wobbleGrabServo")

    @JvmField val intakeTouchSensor      : TouchSensor            = hwInit("intakeTouchSensor")
//    @JvmField val intakeColorSensor      : ColorSensor            = hwInit("intakeColorSensor")
//    @JvmField val intakeDistanceSensor   : DistanceSensor         = hwInit("intakeColorSensor")

    // Odometry module variables - these will be set once we determine plug-in locations on REV Hubs
    override val leftOdometryModule: OdometryModule?  = null
    override val rightOdometryModule: OdometryModule? = null
    override val rearOdometryModule: OdometryModule?  = null

    // LED module - can be uncommented when installed on robot
//    @JvmField val blinkin                 : RevBlinkinLedDriver   = hwInit("blinkin")

    // IMU Controller variables - allows for easier access to get heading
    @JvmField val imuControllerC          : IMUController         = IMUController(hardwareMap = hardwareMap, id = 'C')
    @JvmField val imuControllerE          : IMUController         = IMUController(hardwareMap = hardwareMap, id = 'E')

    // RoadRunner holonomic drivetrain controller and other multi-component systems
     override val holonomicRR             : HolonomicRR           = HolonomicRR(imuControllerC,
                                                                                 frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor,
                                                                                 TwoWheelOdometryLocalizer(odometryLeft, odometryRear, imuControllerC))
    @JvmField val ringDropper             : RingDropper           = RingDropper(ringDropServo)
    @JvmField val intakeSystem            : FullIntakeSystem      = FullIntakeSystem(intakeLiftMotor, liftPotentiometer, ringIntakeMotor, ringDropper, intakeTouchSensor)
    @JvmField val wobbleGrabber           : WobbleGrabber         = WobbleGrabber(wobblePivotServo, wobbleGrabServo)

}
