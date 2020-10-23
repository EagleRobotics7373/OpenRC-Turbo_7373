package org.firstinspires.ftc.teamcode.library.robot.systems.intake

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.library.functions.rhue
import org.firstinspires.ftc.teamcode.library.functions.rsaturation
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.RingDropper
import kotlin.math.absoluteValue

class FullIntakeSystem(
        private val intakeLiftMotor: DcMotorEx,
        private val liftPositionPotentiometer: AnalogInput,
        private val ringIntakeMotor: DcMotorEx,
        private val ringDropServo: RingDropper,
        private val colorSensor: ColorSensor? = null
) {
    var intakeArmState: IntakeArmState = IntakeArmState.IDLE
        private set
    var intakeArmTarget = IntakePosition.GROUND
        private set

    enum class IntakeArmState {
        IDLE, RAISE
    }

    enum class IntakePosition(val ticks: Int, val voltage: Double?) {
        GROUND(0, 1.743),
        WOBBLE(0, 0.0),
        SCORE(430, 0.902)
    }

    var ringIntakeState: RingIntakeState = RingIntakeState.IDLE
        private set
    var desiredRingIntakePower: Double = 1.0
        set(value) {
            field = value.absoluteValue.coerceIn(0.25, 1.0)
        }

    val nextRingIntakeState: RingIntakeState
        get() = when (ringIntakeState) {
            RingIntakeState.IDLE -> RingIntakeState.COLLECT_STAGE_1
            RingIntakeState.COLLECT_STAGE_1 -> RingIntakeState.COLLECT_STAGE_2
            RingIntakeState.COLLECT_STAGE_2 -> RingIntakeState.IDLE_WITH_RING
            RingIntakeState.IDLE_WITH_RING ->
                if (intakeArmTarget != IntakePosition.GROUND) RingIntakeState.OUTPUT
                else RingIntakeState.IDLE_WITH_RING
            RingIntakeState.OUTPUT -> RingIntakeState.IDLE
        }

    var intakeStage1Trigger: (() -> Boolean)? = null
    var intakeStage2Trigger: (() -> Boolean)? = null
    var earliestAllowableAutoStart: Long = Long.MIN_VALUE

    enum class RingIntakeState {
        IDLE, IDLE_WITH_RING, COLLECT_STAGE_1, COLLECT_STAGE_2, OUTPUT
    }

    fun moveIntake(position: IntakePosition) {
        intakeArmState = IntakeArmState.RAISE
        intakeLiftMotor.targetPosition = position.ticks
        intakeArmTarget = position
        intakeLiftMotor.power = if (intakeArmTarget == IntakePosition.GROUND) 0.8 else 1.0
        intakeLiftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    fun manualMoveIntake(power: Double) {
        intakeArmState = IntakeArmState.IDLE
        intakeLiftMotor.power = power.coerceIn(-1.0, 1.0)
        intakeLiftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun doNextRingIntakeState() {
        ringIntakeState = nextRingIntakeState
    }

    fun rejectRing() {
        ringIntakeState = RingIntakeState.OUTPUT
    }

//    fun manualRingMotor(power: Double) {
//        if (power.absoluteValue > 0) {
//            ringIntakeMotor.power = power
////            ringDropServo.pivot(RingDropper.DropperPosition.INTAKE)
//        } else {
//            ringIntakeMotor.power = 0.0
////            ringDropServo.pivot(RingDropper.DropperPosition.HOLD_RING)
//        }
//
//    }

    val ringFullyInIntake: Boolean
    get() = colorSensor?.rhue == 0.0 && colorSensor.rsaturation == 0.0


    fun update() {
        when (intakeArmState) {
            IntakeArmState.IDLE -> {
                intakeLiftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
            IntakeArmState.RAISE -> {
                if (!intakeLiftMotor.isBusy) intakeArmState = IntakeArmState.IDLE
            }
        }

        when (ringIntakeState) {
            RingIntakeState.IDLE -> {
                // Set the intake power to zero and close servo flap
                ringIntakeMotor.power = 0.0
                ringDropServo.pivot(RingDropper.DropperPosition.HOLD_RING)

                // If [intakeStage1Trigger] returns true aka ring is present, start the intake
                if (System.currentTimeMillis() >= earliestAllowableAutoStart
                        && intakeStage1Trigger?.invoke() == true) {
                    doNextRingIntakeState()
                }
            }
            RingIntakeState.COLLECT_STAGE_1 -> {
                // Turn on the intake and put servo flap into intake position
                ringIntakeMotor.power = desiredRingIntakePower
                ringDropServo.pivot(RingDropper.DropperPosition.INTAKE)

                // If [intakeStage2Trigger] returns true aka ring is partially in intake, tighten servo flap
                if (intakeStage2Trigger?.invoke() == true) doNextRingIntakeState()

                // If touch sensor is pressed, ring is fully in intake, so advance to next stage
                if (ringFullyInIntake) doNextRingIntakeState()
            }
            RingIntakeState.COLLECT_STAGE_2 -> {
                // Turn on the intake and tighten servo flap
                ringIntakeMotor.power = desiredRingIntakePower
                ringDropServo.pivot(RingDropper.DropperPosition.HOLD_RING)

                // If touch sensor is pressed, ring is fully in intake, so advance to next stage
                if (ringFullyInIntake) doNextRingIntakeState()
            }
            RingIntakeState.IDLE_WITH_RING -> {
                // Set the intake power to zero and close servo flap
                ringIntakeMotor.power = 0.0
                ringDropServo.pivot(RingDropper.DropperPosition.HOLD_RING)
            }
            RingIntakeState.OUTPUT -> {
                // Turn on the intake, with reversed power to output ring
                ringIntakeMotor.power = -desiredRingIntakePower

                // Pivot ring servo down if on ground, otherwise leave tight
                ringDropServo.pivot(
                        if (intakeArmTarget == IntakePosition.GROUND) RingDropper.DropperPosition.INTAKE
                        else RingDropper.DropperPosition.HOLD_RING
                )

                // Advance the earliest allowable auto-start to two seconds later from now
                earliestAllowableAutoStart = System.currentTimeMillis() + 2000
            }

        }
    }

}