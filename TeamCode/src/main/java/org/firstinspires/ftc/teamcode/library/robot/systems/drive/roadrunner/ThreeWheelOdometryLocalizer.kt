package org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner

import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.OdometryConstants.*
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH
import org.firstinspires.ftc.teamcode.library.functions.roadrunnersupport.Encoder
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.leftOdometryPoseInches
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.rearOdometryPoseInches
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.rightOdometryPoseInches


class ThreeWheelOdometryLocalizer(
        leftModule : Encoder,
        rightModule: Encoder,
        rearModule : Encoder
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

    override fun getWheelPositions(): List<Double> {
        val wheelPositions = modules.map {
            (if (reverseOutput) -1.0 else 1.0) * it.currentPosition.toDouble().encoderTicksToInches()
        }

//        print("%% @OdometryLocalizer_REVExt POS     LEFT=${modules[0].currentPosition}    RIGHT=${modules[1].currentPosition}    REAR=${modules[2].currentPosition}")

        return wheelPositions
    }

    override fun getWheelVelocities(): List<Double>? {
        val wheelVelocities = modules.map {
            (if (reverseOutput) -1.0 else 1.0) * it.correctedVelocity.toDouble().encoderTicksToInches()
        }

//        print("%% @OdometryLocalizer_REVExt VEL     LEFT=${modules[0].velocity}    RIGHT=${modules[1].velocity}    REAR=${modules[2].velocity}")

        return wheelVelocities
    }

    private fun Double.encoderTicksToInches(): Double = (this / ticksPerRevolution) * wheelDiameterMm * Math.PI


}