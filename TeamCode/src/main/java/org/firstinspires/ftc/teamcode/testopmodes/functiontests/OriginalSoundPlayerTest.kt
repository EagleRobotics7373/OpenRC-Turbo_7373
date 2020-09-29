package org.firstinspires.ftc.teamcode.testopmodes.functiontests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.ExtDirMusicPlayer
import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile

@TeleOp(group="Test")
class OriginalSoundPlayerTest: LinearOpMode() {
    override fun runOpMode() {
        val player = ExtDirMusicPlayer(ExtMusicFile.CREEPER_AWMAN)
        waitForStart()
        player.play()
    }
}