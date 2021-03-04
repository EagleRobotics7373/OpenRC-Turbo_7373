package org.firstinspires.ftc.teamcode.opmodes.gen3

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor
import org.firstinspires.ftc.teamcode.library.functions.StartingLine
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BaseRobot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.RobotProvider
import kotlin.math.PI

@Autonomous(name="Curvy Test Auto", group="Gen3 Test")
class CurvyTestAuto: LinearOpMode() {

    lateinit var robot: BaseRobot

    override fun runOpMode() {

        robot = RobotProvider.providePresetRobot(hardwareMap)
        telemetry.addData("Mode", robot.expansionhubs.first().bulkCachingMode)
        telemetry.update()
        robot.expansionhubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.AUTO }
        waitForStart()

        robot.holonomicRR.poseEstimate = Pose2d(
                -63.0,
                -24.0-24.0,
                0.0
        )

        builder(-PI/4)
                .splineToConstantHeading(Vector2d(-13.0, -60.0), 0.0)
                .splineToConstantHeading(Vector2d(15.0, -36.0), PI/2)
                .buildAndRun()

        builder(PI/2)
                .splineTo(Vector2d(-13.0, 0.0), PI)
                .splineTo(Vector2d(-34.0, -22.0), -PI/2)
                .buildAndRun()

        builder(PI/2)
                .splineTo(Vector2d(-13.0, 0.0), 0.0)
                .splineTo(Vector2d(15.0, -36.0), -PI/2)
                .buildAndRun()
    }


    private fun builder() = robot.holonomicRR.trajectoryBuilder()
    private fun builder(tangent: Double) = robot.holonomicRR.trajectoryBuilder(tangent)

    private fun BaseTrajectoryBuilder<TrajectoryBuilder>.buildAndRun(vararg waypointActions: Pair<Double, ()->Unit>) =
            robot.holonomicRR.followTrajectorySync(this.build(), waypointActions.toList())

}