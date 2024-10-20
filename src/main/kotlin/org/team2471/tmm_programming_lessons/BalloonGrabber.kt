package org.team2471.tmm_programming_lessons

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogEncoder
import edu.wpi.first.wpilibj.Relay
import edu.wpi.first.wpilibj.Servo
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.actuators.TalonID
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.CubicSpline
import org.team2471.frc.lib.math.cubicMap
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.Angle.Companion.sin
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.degrees
import org.team2471.tmm_programming_lessons.ClosedLoopPosition.testMotor
import kotlin.math.absoluteValue
import kotlin.math.cos

object BalloonGrabber : Subsystem("BalloonGrabber") {
    val table = NetworkTableInstance.getDefault().getTable(name)

    val leftPitchAngleEntry = table.getEntry("Left Pitch Angle")

    val leftPitchMotor = MotorController(SparkMaxID(Sparks.LEFT_BALLOON_PIVOT, "BalloonGrabber/leftPitchMotor"))
    val leftPitchEncoder = AnalogEncoder(AnalogSensors.LEFT_GRABBER_ENCODDER)
    val leftAirValve = Servo(PWMOutputs.LEFT_AIR_VALVE)

    val leftFans = Relay(PWMOutputs.LEFT_FANS)

    var fansOn = false

    val pitchAngle: Angle
        //      offset
        get() = (239.8 - leftPitchEncoder.get() * 360.0).degrees

    private var pitchSetpoint: Angle = pitchAngle
        set(value) {
            field = value.asDegrees.coerceIn(-50.0, 100.0).degrees
            println("Setpoint: $field")
        }
    val feedForward: Double
        get() = -.04 * sin(pitchAngle)

    val pitchController = PDController(0.01, 0.001)

    init {
        pitchSetpoint = pitchAngle
        GlobalScope.launch {
            periodic {
                leftPitchAngleEntry.setDouble(pitchAngle.asDegrees)
                leftPitchMotor.setPercentOutput(feedForward + pitchController.update((pitchSetpoint - pitchAngle).asDegrees))

                if (fansOn) {
                    leftFans.set(Relay.Value.kReverse)
                } else {
                    leftFans.set(Relay.Value.kForward)
                }
            }
        }
    }

    suspend fun animateToAngle(angle: Angle) {
        val startingAngle = pitchAngle.asDegrees
        var t = 0.0
        var maxTime = (angle.asDegrees - startingAngle).absoluteValue / 180.0
        periodic {
            pitchSetpoint = cubicMap(0.0, maxTime, startingAngle, angle.asDegrees, t).degrees
            t += 0.02
            if (t > maxTime) {
                stop()
            }
        }
    }



    fun balloonIntake() {
        leftAirValve.set(0.3)
    }

    fun balloonRelease() {
        leftAirValve.set(0.0)
    }

    suspend fun pitchCarpetPosition() {
        animateToAngle(100.0.degrees)
    }

    suspend fun pitchTotePosition() {
        animateToAngle(-50.0.degrees)
    }
}