package org.firstinspires.ftc.teamcode.opmodes.gen3

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.functions.StartingLine.*
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBotConstants
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.WobbleGrabber
import org.firstinspires.ftc.teamcode.opmodes.gen1b.OpModeConfig

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "DEMO: Power Shot Test (Deflection)", group = "Main")
class PowerShotTest : LinearOpMode() {

    /*
        VARIABLES: Hardware and Control
     */
    private lateinit var robot           : ExtZoomBot
    private lateinit var imuController   : IMUController

    private          val telem           : MultipleTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private lateinit var elapsedTime     : ElapsedTime

//    private lateinit var cvContainer     : OpenCvContainer<RingContourPipeline>

    private lateinit var player          : ExtDirMusicPlayer

    /*
        VARIABLES: Menu Options
     */
    private val config = OpModeConfig(telemetry)
    private var position: StartingLine by config.custom("Robot Position", CENTER, FAR)
    private var allianceColor: AllianceColor by config.custom("Alliance Color", RED, BLUE)

    override fun runOpMode() {
        /*
            Main autonomous variable initialization
         */
        robot = ExtZoomBot(hardwareMap)
        imuController = robot.imuControllerC
        robot.zoomWheel.mode = DcMotor.RunMode.RUN_USING_ENCODER
        robot.zoomWheel.velocity = 0.0


        /*
            Operate reflective telemetry menu, releasing when OpMode starts or stops
         */
        operateMenu()


        /*
            OpMode has proceeded past init. If stop is requested, return runOpMode() early.
         */
        if (isStopRequested) return


        /*
            Prepare music and setup additional components.
         */
        robot.holonomicRR.redefine()



        robot.zoomWheel.velocity = ExtZoomBotConstants.AUTO_TEST_2
        wait(2000) {updateVelocityToTelemetry()}

        robot.zoomWheel.velocity = ExtZoomBotConstants.AUTO_SERVO_SPEED_1
        robot.deflectionServo.position = ExtZoomBotConstants.AUTO_SERVO_POS_1
        wait(ExtZoomBotConstants.AUTO_SERVO_WAIT.toLong()) {updateVelocityToTelemetry()}
        hit()

        robot.zoomWheel.velocity = ExtZoomBotConstants.AUTO_SERVO_SPEED_2
        robot.deflectionServo.position = ExtZoomBotConstants.AUTO_SERVO_POS_2
        wait(ExtZoomBotConstants.AUTO_SERVO_WAIT.toLong()) {updateVelocityToTelemetry()}
        hit()

        robot.zoomWheel.velocity = ExtZoomBotConstants.AUTO_SERVO_SPEED_3
        robot.deflectionServo.position = ExtZoomBotConstants.AUTO_SERVO_POS_3
        wait(ExtZoomBotConstants.AUTO_SERVO_WAIT.toLong()) {updateVelocityToTelemetry()}
        hit()
        robot.zoomWheel.velocity = 0.0

        /*
            Perform actions
         */

//        val numRings = cvContainer.pipeline.numberOfRings
//        cvContainer.pipeline.tracking = false
//        telemetry.addData("Number of rings", numRings)
//        telemetry.update()

//        doFullAuto(numRings)

        /*
            OpMode actions have finished. Wait until OpMode is stopped, then close resources.
         */
        while (opModeIsActive()) robot.holonomicRR.update()

    }

    fun hit() {
        robot.ringLoadServo.position = ExtZoomBotConstants.RING_LOAD_SERVO_PUSH
        wait(ExtZoomBotConstants.AUTO_SHOOT_WAIT.toLong()) {updateVelocityToTelemetry()}
        robot.ringLoadServo.position = ExtZoomBotConstants.RING_LOAD_SERVO_BACK
        wait(ExtZoomBotConstants.AUTO_SHOOT_WAIT.toLong()) {updateVelocityToTelemetry()}
    }

    /**
     * Creates and operates [ReflectiveTelemetryMenu] before the init period.
     * Controls code until [isStopRequested] or [isStarted] is true.
     */

    fun updateVelocityToTelemetry() {
        telemetry.addData("vel", robot.zoomWheel.velocity)
        telemetry.update()
    }

    fun wait(timeMs: Long, and: ()->Unit) {
        val start = System.currentTimeMillis()
        while (System.currentTimeMillis() < start + timeMs) and()
    }

    fun operateMenu() {

        val dpadUpWatch = ToggleButtonWatcher {gamepad1.dpad_up}
        val dpadDownWatch = ToggleButtonWatcher {gamepad1.dpad_down}
        val dpadLeftWatch = ToggleButtonWatcher {gamepad1.dpad_left}
        val dpadRightWatch = ToggleButtonWatcher {gamepad1.dpad_right}

        config.update()

        while (!isStarted && !isStopRequested) {
            when {
                dpadUpWatch.invoke()    -> config.update(prevItem = true)
                dpadDownWatch.invoke()  -> config.update(nextItem = true)
                dpadLeftWatch.invoke()  -> config.update(iterBack = true)
                dpadRightWatch.invoke() -> config.update(iterFw   = true)
            }
        }

    }

    private fun forDurationMs(duration: Long, action: ()->Unit) {
        val end = System.currentTimeMillis() + duration
        while (System.currentTimeMillis() < end) action.invoke()
    }

    /**
     * Reverses input number if [testColor] matches [allianceColor]
     */
    private infix fun Double.reverseIf(testColor: AllianceColor) : Double =
            if (this@PowerShotTest.allianceColor==testColor) -this else this
}