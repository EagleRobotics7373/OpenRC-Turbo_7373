package org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner

import android.support.annotation.NonNull
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import kotlinx.coroutines.flow.callbackFlow
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.library.functions.roadrunnersupport.DashboardUtil
import org.firstinspires.ftc.teamcode.library.functions.toDegrees
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.baseConstraints
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.headingPID
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.kA
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.kStatic
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.kV
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.trackWidth
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.translationalXPID
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.translationalYPID

//import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.DriveConstantsNew.*
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.DriveConstantsOld.globalPoseEstimate
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.encoderTicksToInches
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.kF
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.motorVelocityPID
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.runUsingEncoder
import kotlin.math.tan


class HolonomicRR

constructor (
             private val imuController: IMUController,
             private val frontLeftExt: DcMotorEx,
             private val backLeftExt:  DcMotorEx,
             private val backRightExt: DcMotorEx,
             private val frontRightExt: DcMotorEx,
             localizer: Localizer)

    : MecanumDrive(kV, kA, kStatic, trackWidth)
{

    private val motorsExt = listOf(frontLeftExt, backLeftExt, backRightExt, frontRightExt)

    private enum class Mode { IDLE, TURN, FOLLOW_TRAJECTORY }
    private var mode = Mode.IDLE

    private val clock = NanoClock.system()
    private val dashboard = FtcDashboard.getInstance()

    private val turnController = PIDFController(headingPID)
    private lateinit var turnProfile : MotionProfile
    private var turnStart = 0.0

    private var driveConstraints = MecanumConstraints(baseConstraints, trackWidth)
    private var follower = HolonomicPIDVAFollower(translationalXPID, translationalYPID, headingPID)

    private lateinit var lastWheelPositions : List<Double>
    private var lastTimestamp = 0.0
    private var lastReadTime : Long = 0

    private var trajectoryStart = 0.0
    private var trajectoryWaypointActions = emptyList<Pair<Double, ()->Unit>>().toMutableList()

    init {
        dashboard.telemetryTransmissionInterval = 25
        turnController.setInputBounds(0.0, 2.0 * Math.PI)

        if (globalPoseEstimate != null) poseEstimate = globalPoseEstimate

//        Thread { while (!Thread.interrupted()) updatePoseEstimate() }.start()

        motorsExt.forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

            if (runUsingEncoder) {
                it.mode = DcMotor.RunMode.RUN_USING_ENCODER
                if (motorVelocityPID != null) setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, motorVelocityPID as PIDCoefficients)
            }
        }

        super.localizer = localizer
    }

//    val trajectoryBuilder : TrajectoryBuilder
//        get() = TrajectoryBuilder(poseEstimate, driveConstraints)

    val lastError: Pose2d
        get() = when(mode) {
            Mode.FOLLOW_TRAJECTORY  -> follower.lastError
            Mode.TURN               -> Pose2d(0.0, 0.0, turnController.lastError)
            Mode.IDLE               -> Pose2d()
        }

    fun update() {
        val beforePoseUpdate = System.currentTimeMillis()
        updatePoseEstimate()
        val afterPoseUpdate = System.currentTimeMillis()

        val currentPose = poseEstimate

        val afterGetPose = System.currentTimeMillis()
        var beforeGetDriveSignal = Long.MIN_VALUE
        var afterGetDriveSignal = Long.MIN_VALUE
        val lastError = this.lastError

        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()

        packet.put("mode", mode)

        val currentHeading = currentPose.heading

        packet.put("x", currentPose.x)
        packet.put("y", currentPose.y)
        packet.put("heading", currentHeading)
        packet.put("heading (deg)", currentHeading.toDegrees())

        packet.put("xError", lastError.x)
        packet.put("yError", lastError.y)
        packet.put("headingError", lastError.heading)

        val imuHeading = rawExternalHeading

        packet.put("imu heading", imuHeading)
        packet.put("imu heading (deg)", imuHeading.toDegrees())

        packet.put("heading diff (deg)", currentHeading.toDegrees() - imuHeading.toDegrees())

        fieldOverlay.setStroke("#3F51B5")
        fieldOverlay.fillCircle(currentPose.x, currentPose.y, 3.0)

        when (mode) {
            Mode.TURN -> {
                val t = clock.seconds() - turnStart

                val targetState = turnProfile[t]

                turnController.targetPosition = targetState.x

                val targetOmega = targetState.v
                val targetAlpha = targetState.a
                val correction = turnController.update(currentPose.heading, targetOmega)

                setDriveSignal(
                        DriveSignal(
                                Pose2d(0.0, 0.0, targetOmega + correction),
                                Pose2d(0.0, 0.0, targetAlpha)
                        )
                )

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())

                }
            }
            Mode.FOLLOW_TRAJECTORY -> {
                val toSet = follower.update(currentPose)
                beforeGetDriveSignal = System.currentTimeMillis()
                setDriveSignal(toSet)
                afterGetDriveSignal = System.currentTimeMillis()

                val trajectory = follower.trajectory

                val currentTrajectoryDuration = clock.seconds() - trajectoryStart
                val totalTrajectoryDuration = trajectory.duration()
                val trajectoryTimeRatio = currentTrajectoryDuration/totalTrajectoryDuration

//                if (trajectoryWaypointActions.isNotEmpty()) {
//                    val currentActionPair = trajectoryWaypointActions.first()
//                    if (trajectoryTimeRatio > currentActionPair.first) {
//                        currentActionPair.second.invoke()
//                        trajectoryWaypointActions.remove(currentActionPair)
//                    }
//                }

                trajectoryWaypointActions.firstOrNull()?.also {
                    if (trajectoryTimeRatio > it.first) {
                        it.second.invoke()
                        trajectoryWaypointActions.remove(it)
                    }
                }

                fieldOverlay.setStrokeWidth(1)
                fieldOverlay.setStroke("#4CAF50")
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.path)

                fieldOverlay.setStroke("#F44336")
                val time = follower.elapsedTime()
                DashboardUtil.drawRobot(fieldOverlay, trajectory[time])

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }
            }

        }

        val endReadTime = System.currentTimeMillis()
        println()
        print("%% HolonomicRR_updates\tREAD GAP = ${beforePoseUpdate - lastReadTime}")
        lastReadTime = endReadTime
        print("\tAFTER POSE UPDATE = ${afterPoseUpdate - beforePoseUpdate}")
        print("\tAFTER GET POSE = ${afterGetPose - beforePoseUpdate}")
        print("\tAFTER UPDATE FOLLOWER = ${beforeGetDriveSignal - beforePoseUpdate}")
        print("\tAFTER GET DRIVE SIGNAL = ${afterGetDriveSignal - beforePoseUpdate}")
        print("\tUPDATE TIME = ${endReadTime - beforePoseUpdate}\t")
        print("\t %% HolonomicRR_pose\t X=${poseEstimate.x}\t Y=${poseEstimate.y}\t HEADING=${currentHeading}\t IMU_HEADING=${imuHeading}\t   XERROR=${lastError.x}\t YERROR=${lastError.y}\t HEADINGERROR=${lastError.heading}")
        dashboard.sendTelemetryPacket(packet)

        globalPoseEstimate = poseEstimate
    }

    @JvmOverloads fun followTrajectory(trajectory: Trajectory, waypointActions: List<Pair<Double, ()->Unit>> = emptyList()) {
        follower.followTrajectory(trajectory)
        lastReadTime = System.currentTimeMillis()
//        driveSignalUpdateThread = Thread(driveSignalUpdateRunnable)
//        driveSignalUpdateThread.start()
        mode = Mode.FOLLOW_TRAJECTORY
        trajectoryStart = clock.seconds()
        trajectoryWaypointActions = waypointActions.sortedBy { it.first }.toMutableList()
    }

    @JvmOverloads fun followTrajectorySync(trajectory: Trajectory, waypointActions: List<Pair<Double, ()->Unit>> = emptyList()) {
        followTrajectory(trajectory, waypointActions)
        waitForIdle()
    }

    fun turn(angle: Double) {
        val heading = poseEstimate.heading

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(heading, 0.0, 0.0, 0.0),
                MotionState(heading + angle, 0.0, 0.0, 0.0),
                driveConstraints.maxAngVel,
                driveConstraints.maxAngAccel,
                driveConstraints.maxAngJerk
        )

        turnStart = clock.seconds()
        lastReadTime = System.currentTimeMillis()
//        driveSignalUpdateThread = Thread(driveSignalUpdateRunnable)
//        driveSignalUpdateThread.start()
        mode = Mode.TURN
    }

    fun turnSync(angle: Double) {
        turn(angle)
        waitForIdle()
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy()) update()
    }

    fun isBusy() : Boolean {
        return mode != Mode.IDLE
    }

    fun getPIDCoefficients(runMode: DcMotor.RunMode) : PIDCoefficients {
        val pidfCoefficients = (frontLeftExt as DcMotorEx).getPIDFCoefficients(runMode)
        return PIDCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d)
    }

    fun setPIDCoefficients(runMode: DcMotor.RunMode, coefficients: PIDCoefficients) {
        motorsExt.forEach {
            (it as DcMotorEx).setPIDFCoefficients(runMode, PIDFCoefficients(coefficients.kP, coefficients.kI, coefficients.kD, kF))
            // set kF to motor velocity F coefficient, look at RR quickstart, DriveConstantsNew
        }
    }

    @NonNull
    override fun getWheelPositions() : List<Double> {
        return listOf(
                encoderTicksToInches(frontLeftExt.currentPosition.toDouble()),
                encoderTicksToInches(backLeftExt.currentPosition.toDouble()),
                encoderTicksToInches(backRightExt.currentPosition.toDouble()),
                encoderTicksToInches(frontRightExt.currentPosition.toDouble())
        )
    }

    @NonNull
    fun getWheelVelocities() : List<Double> {
        return listOf(
                encoderTicksToInches(frontLeftExt.velocity),
                encoderTicksToInches(backLeftExt.velocity),
                encoderTicksToInches(backRightExt.velocity),
                encoderTicksToInches(frontRightExt.velocity)
        )
    }

    override val rawExternalHeading: Double
        get() = imuController.getHeading()

    override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
        frontLeftExt .power =  frontLeft
        backLeftExt  .power =  rearLeft
        backRightExt .power = -rearRight
        frontRightExt.power = -frontRight
    }

    fun redefine() {
        follower = HolonomicPIDVAFollower(translationalXPID, translationalYPID, headingPID)
        driveConstraints = MecanumConstraints(baseConstraints, trackWidth)
    }

    @JvmOverloads fun trajectoryBuilder(tangent : Double, _driveConstraints : DriveConstraints = driveConstraints) = TrajectoryBuilder(poseEstimate.copy(heading = tangent), startHeading = poseEstimate.heading, constraints = _driveConstraints)
    @JvmOverloads fun trajectoryBuilder(_driveConstraints : DriveConstraints = driveConstraints) = TrajectoryBuilder(poseEstimate, constraints = _driveConstraints)


}