package org.team2471.frc2024

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.jetbrains.org.objectweb.asm.tree.analysis.Value
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion.following.demoMode
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2024.Drive.isRedAlliance
import org.team2471.frc2024.Pivot.MAXHARDSTOP
import org.team2471.frc2024.Pivot.MINHARDSTOP
import org.team2471.frc2024.Pivot.feedForward
import org.team2471.frc2024.Pivot.pivotEncoderAngle
import org.team2471.frc2024.Pivot.pivotMotor
import javax.swing.text.Position

// TMM LESSON COMMENT:  problem 2 part 2 - control the motors position just like part 1, but use the motor controller instead

// steps:
// remove the PDController vars
// in the setpoint set() functions call each motor's  setPositionSetpoint() functions


object Shooter: Subsystem("Shooter") {

    // this creates a network table for the Shooter subsystem
    private val table = NetworkTableInstance.getDefault().getTable("Shooter")

    // these are entries in our table for the network table
    private val motorRpmBottomEntry = table.getEntry("RPM Bottom")
    private val motorRpmTopEntry = table.getEntry("RPM Top")

    const val MAXRPM = 5800.0


    // TMM LESSON COMMENT:  Note how MotorControllers are made below, and where it gets their CAN bus id from RobotMap.kt

    val shooterMotorBottom = MotorController(FalconID(Falcons.SHOOTER_BOTTOM, "Shooter/Top"))
    val shooterMotorTop = MotorController(FalconID(Falcons.SHOOTER_TOP, "Shooter/Bottom"))

    // Problem 2 objective is to control the motor by using its encoder and software functions from meanlib


    // add a val to store the angle of the encoder, see 2024 pivot for an example
    val motorpositionTop: Angle
        get() = shooterMotorTop.position.degrees

    val motorpositionBottom: Angle
        get() = shooterMotorBottom.position.degrees


    // add a var to store the motor angle setpoint

    var positionTopSetpoint = motorpositionTop
        set(value) {
            field = value.asDegrees.coerceIn(0.0, 90.0).degrees
        }

    var positionBottomSetpoint = motorpositionBottom
        set(value) {
            field = value.asDegrees.coerceIn(0.0, 90.0).degrees

        }

    // create a val of type PDController from meanlib

    val topPDController = PDController(0.0005, 0.0)
    val bottomPDController = PDController(0.0005, 0.0)

    val motorRpmTop
        get() = shooterMotorTop.velocity

    val motorRpmBottom
        get() = shooterMotorBottom.velocity

    var rpmTopSetpoint = 0.0
    var rpmBottomSetpoint = 0.0

    val averageRpm
        get() = (motorRpmBottom + motorRpmTop) / 2.0

    val averageRpmSetpoint
        get() = (rpmTopSetpoint + rpmBottomSetpoint) / 2.0

    var manualShootState = false

    val pitchCurve = MotionCurve()
    val rpmCurve = MotionCurve()

    init {

        shooterMotorBottom.config {
            feedbackCoefficient = 360.0 / 1.0
            currentLimit(30, 40, 1.0)
            coastMode()
            inverted(true)
            configSim(DCMotor.getKrakenX60Foc(1), 0.005)
        }

        shooterMotorTop.config {
            feedbackCoefficient = 360.0 / 1.0
            currentLimit(30, 40, 1.0)
            coastMode()
            inverted(true)
            configSim(DCMotor.getKrakenX60Foc(1), 0.005)
        }


        GlobalScope.launch {
            periodic {
                motorRpmBottomEntry.setDouble(motorRpmBottom)
                motorRpmTopEntry.setDouble(motorRpmTop)

                // calculate the error between angleSetpoint and motorAngle and store it in a val
                val topError = (positionTopSetpoint - motorpositionTop).asDegrees
                val bottomError = (positionBottomSetpoint - motorpositionBottom).asDegrees

                // update the PDController and set the motor power with the result
                val topPower = topPDController.update(topError)
                val bottomPower = bottomPDController.update(bottomError)

                shooterMotorTop.setPercentOutput(topPower)
                shooterMotorBottom.setPercentOutput(bottomPower)
            }
        }
    }

    override suspend fun default() {
        periodic {
        }
    }

    override fun onDisable() {
    }

    fun setRpms(rpm: Double) {

    }

    fun setPower(power: Double) {
        // TMM LESSON COMMENT: call setPercentOutput here for both the top and bottom motors
        shooterMotorTop.setPercentOutput(power)
        shooterMotorBottom.setPercentOutput(power)
    }

    // make two suspending functions to map to buttons in OI which set the angle setpoint to different targets and add a print to each one
    suspend fun positionA() {
        positionTopSetpoint = 90.0.degrees
        positionBottomSetpoint = 90.0.degrees
        println("position = 90")
    }

    suspend fun positionB() {
        positionTopSetpoint = 0.0.degrees
        positionBottomSetpoint = 0.0.degrees
        println("position = 0")
    }
}