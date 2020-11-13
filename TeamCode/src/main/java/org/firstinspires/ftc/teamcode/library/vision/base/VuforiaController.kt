package org.firstinspires.ftc.teamcode.library.vision.base

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener
import org.firstinspires.ftc.teamcode.library.functions.Point3D

class VuforiaController(private val vuforia: VuforiaLocalizer, private val telemetry: Telemetry) {
    // We will define some constants and conversions here
    private val mmPerInch = 25.4f
    private val mmTargetHeight = 6 * mmPerInch          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private val stoneZ = 2.00f * mmPerInch

    // Class Members
    private var lastLocation: OpenGLMatrix? = null
    private var targetVisible = false
    private var phoneXRotate = 0f
    private var phoneYRotate = 0f
    private val phoneZRotate = 0f

    private val targetsSkyStone = this.vuforia.loadTrackablesFromFile("/sdcard/FIRST/Skystone")
    private val stoneTarget = targetsSkyStone[0]

    init {
        stoneTarget.location = (OpenGLMatrix
                .translation(0f, 0f, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90f, 0f, -90f)))
    }

    fun activate() = targetsSkyStone.activate()

    fun analyzeVuforiaResult() : Point3D? {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false
        val trackable = stoneTarget
        if ((trackable.getListener() as VuforiaTrackableDefaultListener).isVisible) {
            telemetry.addData("Visible Target", trackable.getName())
            targetVisible = true

            // getUpdatedRobotLocation() will return null if no new information is available since
            // the last time that invoke was made, or if the trackable is not currently visible.
            val robotLocationTransform = (trackable.getListener() as VuforiaTrackableDefaultListener).updatedRobotLocation
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform
            }
        }
        var point : Point3D? = null
        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            val translation = lastLocation!!.getTranslation()
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch)

            // express the rotation of the robot in degrees.
            val rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES)
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle)
            point = Point3D(translation[0].toDouble(),translation[1].toDouble(), translation[2].toDouble())
        } else {
            telemetry.addData("Visible Target", "none")

        }
        telemetry.update()
        return point
    }

    fun deactivate() = targetsSkyStone.deactivate()

}