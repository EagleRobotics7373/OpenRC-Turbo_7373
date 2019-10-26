package org.firstinspires.ftc.teamcode.library.functions

import android.media.MediaPlayer
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.R

class ExtDirMusicPlayer(extMusicFile: ExtMusicFile, seekAtBeginning: Boolean = false) {

    constructor(extMusicFile: ExtMusicFile):this(extMusicFile, false)

    private val mediaPlayer: MediaPlayer = MediaPlayer()
    var fileIsCorrect = false
        private set
    init {
        mediaPlayer.isLooping = true
        try {
            mediaPlayer.setDataSource("/sdcard/FIRST/music/"+extMusicFile.fileName)
            mediaPlayer.prepare()
            mediaPlayer.seekTo(if (seekAtBeginning) extMusicFile.beginSeekTo else 0)
            fileIsCorrect = true
        } catch (e: Exception) {
            fileIsCorrect = false
        }
    }

    fun play(): Boolean {
        try {
            if (!mediaPlayer.isPlaying) {
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

enum class ExtMusicFile(@JvmField val fileName: String, @JvmField val beginSeekTo: Int = 0) {
    UNITY("unity.mp3"),
    MEGALOUNITY("megalounity.mp3"),
    CRABRAVE("crabrave.mp3", 59500),
    BRADTHECHEMIST("bradthechemist.mp3"),
    PACMAN("pacman.mp3"),
    MEGALOVANIA("megalovania.mp3", 16000),
    TETRIS("tetris.mp3"),
    NONE("none")
}