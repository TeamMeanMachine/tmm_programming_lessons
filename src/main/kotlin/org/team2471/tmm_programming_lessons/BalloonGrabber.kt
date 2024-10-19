package org.team2471.tmm_programming_lessons

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogEncoder
import edu.wpi.first.wpilibj.Servo
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.TalonID
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import org.team2471.tmm_programming_lessons.ClosedLoopPosition.testMotor

object BalloonGrabber : Subsystem("BalloonGrabber") {
    val table = NetworkTableInstance.getDefault().getTable(name)

    val leftPitchMotor = MotorController(TalonID(Talons.LEFT_GRABBER_PITCH, "BalloonGrabber/leftPitchMotor"))
    val leftPitchEncoder = AnalogEncoder(AnalogSensors.LEFT_GRABBER_ENCODDER)
    val leftAirValve = Servo(PWMOutputs.LEFT_AIR_VALVE)

    val pitchAngle: Angle
        get() = ((leftPitchEncoder.get() - 0.2) / 4.6).degrees

    var pitchSetpoint: Angle = pitchAngle
        set(value) {
            field = value.asDegrees.coerceIn(-120.0, 120.0).degrees
        }

    val pitchController = PDController(0.02, 0.0)

    init {
        GlobalScope.launch {
            periodic {
                leftPitchMotor.setPercentOutput(pitchController.update((pitchSetpoint - pitchAngle).asDegrees))
            }
        }
    }

    fun balloonIntake() {
        leftAirValve.set(0.0)
    }

    fun balloonRelease() {
        leftAirValve.set(90.0)
    }

    fun pitchCarpetPosition() {
        pitchSetpoint = 120.0.degrees
    }

    fun pitchTotePosition() {
        pitchSetpoint = -100.0.degrees
    }
}