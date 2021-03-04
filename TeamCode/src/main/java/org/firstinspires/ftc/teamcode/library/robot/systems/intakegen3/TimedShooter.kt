package org.firstinspires.ftc.teamcode.library.robot.systems.intakegen3

import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBotConstants

class TimedShooter() {
    lateinit var robot : ExtZoomBot

    private var lastChange: Long = System.currentTimeMillis()
    private val timeSinceLastChange: Long get() = System.currentTimeMillis() - lastChange

}