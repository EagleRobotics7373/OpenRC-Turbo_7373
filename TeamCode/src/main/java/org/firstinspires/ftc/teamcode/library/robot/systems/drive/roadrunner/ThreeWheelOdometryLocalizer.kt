package org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner

import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.OdometryConstants.*
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.leftOdometryPoseInches
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.rearOdometryPoseInches
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.rightOdometryPoseInches


class ThreeWheelOdometryLocalizer(
        leftModule : DcMotorEx,
        rightModule: DcMotorEx,
        rearModule : DcMotorEx
)
    : ThreeTrackingWheelLocalizer(
        listOf(
//                Pose2d(INCH.fromCm(leftXcm), INCH.fromCm(leftYcm), leftAngleDeg.toRadians()),
//                Pose2d(INCH.fromCm(rightXcm), INCH.fromCm(rightYcm), rightAngleDeg.toRadians()),
//                Pose2d(INCH.fromCm(rearXcm), INCH.fromCm(rearYcm), rearAngleDeg.toRadians())
                leftOdometryPoseInches,
                rightOdometryPoseInches,
                rearOdometryPoseInches
        )
)
{
    private val modules = listOf(leftModule, rightModule, rearModule)

    private val ticksPerRevolution = 8192
    private val wheelDiameterMm = INCH.fromMm(38.0)

    init {
        modules.forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    override fun getWheelPositions(): List<Double> {
        val wheelPositions = modules.map {
            (if (reverseOutput) -1.0 else 1.0) * (it.currentPosition.toDouble() / ticksPerRevolution) * wheelDiameterMm * Math.PI
        }

        print("%% @OdometryLocalizer_REVExt   LEFT=${wheelPositions[0]}    RIGHT=${wheelPositions[1]}    REAR=${wheelPositions[2]}")

        return wheelPositions
    }


}