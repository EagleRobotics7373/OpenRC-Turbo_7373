package org.firstinspires.ftc.teamcode.testopmodes.exceptiontests

import android.app.Activity
import android.app.AlarmManager
import android.app.PendingIntent
import android.content.Context
import android.content.Intent
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl
import org.firstinspires.ftc.robotcore.internal.system.RobotApplication
import kotlin.system.exitProcess

class ExceptionHandler(private val activity: Activity) : Thread.UncaughtExceptionHandler {

    override fun uncaughtException(t: Thread?, e: Throwable?) {
        val intent = Intent(activity, FtcRobotControllerActivity::class.java)
        intent.putExtra("crash", true)
        intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP or Intent.FLAG_ACTIVITY_CLEAR_TASK or Intent.FLAG_ACTIVITY_NEW_TASK)

        val manager = OpModeManagerImpl.getOpModeManagerOfActivity(activity)
        val context = manager.hardwareMap.appContext
        val pendingIntent = PendingIntent.getActivity(context, 0, intent, PendingIntent.FLAG_ONE_SHOT)

        val mgr = context.getSystemService(Context.ALARM_SERVICE) as AlarmManager
        mgr.set(AlarmManager.RTC, System.currentTimeMillis(), pendingIntent)

        print("\n\n\n\n\n\n\n CRASHING due to uncaught exception \n\n\n\n\n")
        exitProcess(2)
    }

}