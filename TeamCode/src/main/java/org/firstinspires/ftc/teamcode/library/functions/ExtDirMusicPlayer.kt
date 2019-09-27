package org.firstinspires.ftc.teamcode.library.functions

import android.media.MediaPlayer
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.R

class ExtDirMusicPlayer {

    private val mediaPlayer: MediaPlayer = MediaPlayer()

    init {
        mediaPlayer.isLooping = true
        mediaPlayer.seekTo(0)
    }

    fun play(extMusicFile: ExtMusicFile): Boolean {
        try {
            if (!mediaPlayer.isPlaying) {
                mediaPlayer.setDataSource("/sdcard/FIRST/music/"+extMusicFile.fileName)
                mediaPlayer.prepare()
                mediaPlayer.start()
            }
        } catch (e : Exception) {
            return false
        }
        return true
    }

    fun pause() {
        mediaPlayer.pause()
    }

    fun stop() {
        mediaPlayer.stop()
        mediaPlayer.release()
    }

    fun isPlaying() = mediaPlayer.isPlaying
}

enum class ExtMusicFile(@JvmField val fileName: String) {
    UNITY("unity.mp3"),
    MEGALOUNITY("megalounity.mp3"),
    BRADTHECHEMIST("bradthechemist.mp3")
}