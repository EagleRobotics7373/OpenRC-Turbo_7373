package org.firstinspires.ftc.teamcode.library.robot.systems.intake

import com.acmerobotics.roadrunner.profile.MotionProfileBuilder
import com.acmerobotics.roadrunner.profile.MotionState
import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo

class FullIntakeSystem(
        private val intakeLiftMotor: DcMotorEx,
        private val liftPositionPotentiometer: AnalogInput,
        private val ringIntakeMotor: DcMotorEx,
        private val ringDropServo: Servo
) {
    var intakeState: State = State.IDLE
    val minIntakeVoltage = 0.0
    val maxIntakeVoltage = 3.0

    enum class State {
        IDLE, MANUAL, AUTOMATED
    }

    enum class IntakePosition(val voltage: Double) {
        COLLECT(0.0),
        WOBBLE(0.0),
        SCORE(0.0)
    }

    fun moveIntake(position: IntakePosition) {

    }

    fun manualMoveIntake(power: Double) {
        intakeLiftMotor.power = power.coerceIn(-1.0, 1.0)
    }

    fun manualRingMotor(power: Double) {
        ringIntakeMotor.power = power
    }

    fun ringServoRelease() {
        ringDropServo.position = 0.0
    }

    fun ringServoGrab() {
        ringDropServo.position = 0.0
    }



    fun update() {
        when (intakeState) {
            State.IDLE -> {

            }
        }
    }

}