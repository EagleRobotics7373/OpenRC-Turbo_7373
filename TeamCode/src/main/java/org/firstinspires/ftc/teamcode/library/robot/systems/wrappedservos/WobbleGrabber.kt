package org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos

import com.qualcomm.robotcore.hardware.Servo

class WobbleGrabber
constructor (private val pivotServo : Servo,
             private val grabServo  : Servo)
{

    /**
        Enum class defining positions for the arm to pivot
     */
    enum class PivotPosition(val position: Double) {
        GRAB(0.02),
        PERPENDICULAR(0.07),
        OVER_WALL(0.24),
        VERTICAL(0.48),
        YEET(0.68),
        STORAGE(0.70)
    }

    /**
        Enum class defining positions for the grabber to pivot
     */
    enum class GrabPosition(val position: Double) {
        GRAB(0.01),
        MID_GRAB(0.33),
        STORAGE(0.69)
    }

    /**
     * Pivot the wobble grabber arm to a specified position
     * @param loc the position to pivot
     */
    private fun pivot(loc: PivotPosition) { pivotServo.position = loc.position }

    /**
     * Pivot the wobble grabber end effector to a specified position
     * @param loc the position to pivot the grabber
     */
    private fun grab(loc: GrabPosition) { grabServo.position = loc.position }

    fun move(grab: GrabPosition? = null, pivot: PivotPosition? = null) {
        state = WobbleGrabberState.CUSTOM
        if (grab != null)  grab(grab)
        if (pivot != null) pivot(pivot)
    }

    /**
        Describes various states for the wobble grabber for easier gamepad-based control
     */
    enum class WobbleGrabberState(
            val action: ((WobbleGrabber, Long)->Unit)?,
            val prev: ()->WobbleGrabberState?,
            val next: ()->WobbleGrabberState?)
    {

        STORAGE(
                action = { it, _ -> it.pivot(PivotPosition.YEET); it.grab(GrabPosition.STORAGE) },
                prev = { RELEASE },
                next = { GRAB_PREP }),
        GRAB_PREP(
                action = { it, _ -> it.pivot(PivotPosition.GRAB); it.grab(GrabPosition.MID_GRAB) },
                prev = { STORAGE },
                next = { GRAB }),
        GRAB(
                action = { it, _ -> it.pivot(PivotPosition.GRAB); it.grab(GrabPosition.GRAB) },
                prev = { GRAB_PREP },
                next = { IN_AIR }),
        IN_AIR(
                action = { it, _ -> it.pivot(PivotPosition.VERTICAL); it.grab(GrabPosition.GRAB) },
                prev = { GRAB },
                next = { RELEASE }),
        RELEASE(
                action = { it, time -> it.pivot(PivotPosition.OVER_WALL); it.grab(if (time > org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBotConstants.WOBBLE_RELEASE_DELAY) GrabPosition.MID_GRAB else GrabPosition.GRAB) },
                prev = { IN_AIR },
                next = { STORAGE  }),
        CUSTOM(
                action = null,
                prev = { STORAGE },
                next = { STORAGE })

    }

    /**
     * Records the current state of the wobble grabber
     */
    var state: WobbleGrabberState = WobbleGrabberState.CUSTOM
    set(value) { stateBeginTime = System.currentTimeMillis(); value.action?.invoke(this, 0); field = value; }

    /**
     * Records the time at which a state is changed
     */
    var stateBeginTime: Long = System.currentTimeMillis()

    /**
     * Reports the duration in which a certain state has been active
     * Subtracts the result of [System.currentTimeMillis] from [stateBeginTime]
     */
    val stateDuration: Long get() = System.currentTimeMillis() - stateBeginTime

    /**
     * Quick-access function for going to the next state
     */
    fun prevState() { state = state.prev() ?: state }

    /**
     * Quick-access function for returning to the previous state
     */
    fun nextState() { state = state.next() ?: state }

    /**
     * Re-run the state action function to allow for time-delayed actions
     */
    fun update() { state.action?.invoke(this, stateDuration) }

}