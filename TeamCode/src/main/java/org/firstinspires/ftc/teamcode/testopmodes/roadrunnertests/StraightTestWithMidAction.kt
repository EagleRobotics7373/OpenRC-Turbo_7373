package org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtRingPlaceBot
import org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests.RoadRunnerTestConstants.STRAIGHT_TEST_DIST

@TeleOp(group="rr_cfg")
class StraightTestWithMidAction : LinearOpMode() {
    override fun runOpMode() {
        val robot = ExtRingPlaceBot(hardwareMap)
        val drive = robot.holonomicRR
        waitForStart()

        if (isStopRequested) return

        val trajectory =
                drive.trajectoryBuilder()
                        .forward(STRAIGHT_TEST_DIST)
                        .build()

        drive.followTrajectorySync(trajectory, listOf(Pair(0.5, {print("This is a mid action!")})))
        while (!isStopRequested) drive.update()
    }

}