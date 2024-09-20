@file:JvmName("Main")

package org.team2471.tmm_programming_lessons

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.team2471.frc.lib.framework.LoggedMeanlibRobot
import org.team2471.frc.lib.motion.following.demoMode
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.RobotMode
import org.team2471.frc.lib.util.robotMode
import org.team2471.tmm_programming_lessons.testing.*
import java.net.NetworkInterface




@DelicateCoroutinesApi
object Robot : LoggedMeanlibRobot() {
    var startMeasureTime = System.nanoTime()
    var lastMeasureTime = startMeasureTime
    var isCompBot = true
    init {
        val networkInterfaces =  NetworkInterface.getNetworkInterfaces()
        println("retrieving network interfaces")
        for (iFace in networkInterfaces) {
            println("${iFace.name}")
            if (iFace.name == "eth0" && !isSimulation()) {
                println("NETWORK NAME--->${iFace.name}<----")
                var macString = ""
                for (byteVal in iFace.hardwareAddress){
                    macString += String.format("%s", byteVal)
                }
                println("FORMATTED---->$macString<-----")

                isCompBot = (macString != "0-12847512372")
                println("I am compbot = $isCompBot")
            }
        }

        if (robotMode != RobotMode.REPLAY) {
            //sim or real
//            Logger.addDataReceiver(WPILOGWriter())
            Logger.addDataReceiver(NT4Publisher())
        } else {
            setUseTiming(true) // false = run sim as fast as possible
            val logPath = LogFileUtil.findReplayLog()
            Logger.setReplaySource(WPILOGReader(logPath))
            Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
        }

        Logger.start()

        // i heard the first string + double concatenations were expensive...
        repeat(25) {
            println("RANDOM NUMBER: ${Math.random()}")
        }
        println("NEVER GONNA GIVE YOU UP")

        FieldManager
        OI
        println("Field Manager Active! Is Red ${FieldManager.isRedAlliance}")
        Drive
        println("Activating Drive!")
        Drive.zeroGyro()
        Drive.heading = 0.0.degrees

//        PoseEstimator
//        println("Activating PoseEstimator! currentPose ${PoseEstimator.currentPose}")

    }

    override suspend fun enable() {
        println("starting enable")
        FieldManager.beforeFirstEnable = false
        Drive.enable()
        println("field centric? ${SmartDashboard.getBoolean("Use Gyro", true) && !DriverStation.isAutonomous()}")
        println("ending enable")
    }

    override suspend fun autonomous() {
        if (!Drive.demoMode) {
            initTimeMeasurement()
            println("autonomous starting")
            Drive.brakeMode()
            Drive.aimPDController = Drive.autoPDController
            println("autonomous Drive brakeMode ${totalTimeTaken()}")
            println("autonomous ending ${totalTimeTaken()}")
        } else {
            println("CANNOT RUN AUTO IN DEMO MODE!!!!!! (you're welcome for not killing anyone)")
        }
    }

    override suspend fun teleop() {
        println("telop begin")
        Drive.aimPDController = Drive.teleopPDController
        Drive.headingSetpoint = Drive.heading
    }

    override suspend fun test()  {
        println("test mode begin. Hi.")
//        Intake.pidTestOne()
//        pathFollowTest()
        Drive.setAngleOffsets()
//        useTest()
//        Drive.steeringTests()
        println("finished testing")
    }


    override suspend fun disable() {
        OI.driverController.rumble = 0.0
        OI.operatorController.rumble = 0.0
        Drive.disable()
    }

    private fun initTimeMeasurement(){
        startMeasureTime = System.nanoTime()
        lastMeasureTime = startMeasureTime
    }

    private fun updateNanosTaken(){
        lastMeasureTime = System.nanoTime()
    }

    fun totalTimeTaken(): Long {
        return System.nanoTime() - startMeasureTime
    }

    fun recentTimeTaken(): Long {
        val timeTaken = System.nanoTime() - lastMeasureTime
        updateNanosTaken()
        return timeTaken
    }
}

fun main() {
    println("start robot")
    RobotBase.startRobot { Robot }
}
