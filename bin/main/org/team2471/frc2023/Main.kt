@file:JvmName("Main")

package org.team2471.tmm_programming_lessons

import FRC____.BuildConfig
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import org.team2471.frc.lib.framework.MeanlibRobot
import org.team2471.frc.lib.units.degrees
import org.team2471.tmm_programming_lessons.testing.*
import java.net.NetworkInterface




@DelicateCoroutinesApi
object Robot : MeanlibRobot() {
    var startMeasureTime = System.nanoTime()
    var lastMeasureTime = startMeasureTime
    var isCompBot = true
    init {
        val networkInterfaces =  NetworkInterface.getNetworkInterfaces()
        println("retrieving network interfaces")
        for (iFace in networkInterfaces) {
            println("${iFace.name}")
            if (iFace.name == "eth0") {
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

        // i heard the first string + double concatenations were expensive...
        repeat(25) {
            println("RANDOM NUMBER: ${Math.random()}")
        }
        println("NEVER GONNA GIVE YOU UP")
        println(BuildConfig.BUILD_TIME)
        FieldManager
        OI
        Drive.zeroGyro()
        Drive.heading = 0.0.degrees

        AutoChooser
        NodeDeckHub
//        AprilTag
        Intake
        Arm
        PowerInfo
    }

    override suspend fun enable() {
        println("starting enable")
        Drive.enable()
        Arm.enable()
        Intake.enable()
        PowerInfo.enable()
//        SignalLights.enable()
        println("field centric? ${SmartDashboard.getBoolean("Use Gyro", true) && !DriverStation.isAutonomous()}")
        println("ending enable")
    }

    override suspend fun autonomous() {
       initTimeMeasurement()
        println("autonomous starting")
        Drive.brakeMode()
        Drive.aimPDController = Drive.autoPDController
        println("autonomous Drive brakeMode ${totalTimeTaken()}")
        AutoChooser.autonomous()
        println("autonomous ending ${totalTimeTaken()}")
    }

    override suspend fun teleop() {
        println("telop begin")
        Drive.aimPDController = Drive.teleopPDController
        Drive.headingSetpoint = Drive.heading
    }

    override suspend fun test()  {
        println("test mode begin. Hi.")
        Arm.feedForwardTest()
//        Drive.setAngleOffsets()
    }


    override suspend fun disable() {
        Drive.disable()
        Arm.disable()
        Intake.disable()
        PowerInfo.disable()
//        SignalLights.disable()
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
