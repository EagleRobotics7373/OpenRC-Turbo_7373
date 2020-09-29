package org.firstinspires.ftc.teamcode.testopmodes.functiontests

import com.qualcomm.ftccommon.SoundPlayer
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.ExtDirMusicPlayer
import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile
import java.io.File

@TeleOp(group = "Test")
class SoundPlayerTest : LinearOpMode() {
    override fun runOpMode() {
        val context = hardwareMap.appContext
        val player = SoundPlayer.getInstance()
        val musicFile = ExtMusicFile.CREEPER_AWMAN
        val musicFileName = "/sdcard/FIRST/music/${musicFile.fileName}"
        val playingParams = SoundPlayer.PlaySoundParams().also {
            it.volume = 1.0f
            it.loopControl = 0
            it.rate = 1.0f
        }
        player.preload(context, File(musicFileName))
        telemetry.addLine("Waiting for start")
        telemetry.update()
        waitForStart()



//        player.startPlaying(context, File(musicFileName), playingParams, null, null)

        player.play(context, File(musicFileName), playingParams.volume, playingParams.loopControl, playingParams.rate)

        while (opModeIsActive()) {
            telemetry.addData("Runtime", this.runtime)
            telemetry.addData("Local Sound On", player.isLocalSoundOn)
            telemetry.update()
        }
        player.stopPlayingLoops()
        player.stopPlayingAll()
    }
}