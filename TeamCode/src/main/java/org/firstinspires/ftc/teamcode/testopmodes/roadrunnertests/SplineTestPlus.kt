package org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.toRadians
import org.firstinspires.ftc.teamcode.library.robot.robotcore.RobotProvider.providePresetRobot
import org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests.RoadRunnerTestConstants.*

@TeleOp(group="rr_cfg")
class SplineTestPlus : LinearOpMode() {
    override fun runOpMode() {
        val robot = providePresetRobot(hardwareMap)
        val drive = robot.holonomicRR
        waitForStart()
        val splineTestEndPose = Pose2d(
                SPLINE_TEST_X,
                SPLINE_TEST_Y,
                SPLINE_TEST_ENDTANGENT.toRadians()
        )
        val trajectory =
                drive.trajectoryBuilder(tangent = SPLINE_TEST_STARTTANGENT.toRadians())
                        .build()

        if (isStopRequested) return

        drive.followTrajectorySync(trajectory)
        while (!isStopRequested) drive.update()
    }

}