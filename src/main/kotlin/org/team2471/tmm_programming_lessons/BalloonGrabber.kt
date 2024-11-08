package org.team2471.tmm_programming_lessons

import com.revrobotics.ColorSensorV3
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogEncoder
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.PWM
import edu.wpi.first.wpilibj.Relay
import edu.wpi.first.wpilibj.Servo
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.delay
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
import org.team2471.frc.lib.coroutines.parallel
import org.team2471.frc.lib.coroutines.suspendUntil
import org.team2471.frc.lib.util.Timer


object BalloonGrabber : Subsystem("BalloonGrabber") {
    val table = NetworkTableInstance.getDefault().getTable(name)

    val leftPitchAngleEntry = table.getEntry("Left Pitch Angle")
    val encoderTicksEntry = table.getEntry("Encoder Ticks")
    val encoderTicksOffsetEntry = table.getEntry("Encoder Ticks Offset")

    val leftPitchMotor = MotorController(SparkMaxID(Sparks.LEFT_BALLOON_PIVOT, "BalloonGrabber/leftPitchMotor"))
    val leftPitchEncoder = AnalogEncoder(AnalogSensors.LEFT_GRABBER_ENCODDER)
    val leftAirValve = Servo(PWMOutputs.LEFT_AIR_VALVE)

    //val leftFans = Relay(PWMOutputs.LEFT_FANS)
//    val leftFans = MotorController(TalonID(36, "hi there"))
    val leftFans = PWM(PWMOutputs.LEFT_FANS)

    private val i2cPort: I2C.Port = I2C.Port.kMXP
    private val colorSensor = ColorSensorV3(i2cPort)


    val CARPET_ANGLE = -38.0.degrees
    val TOTE_ANGLE = (180.0).degrees

    var fansOn = false

    val encoderRawTicks: Double
        get() = leftPitchEncoder.get() * 360.0
    var encoderTicksOffset: Double
        get() = encoderTicksOffsetEntry.getDouble(0.616)
        set(value) {
            encoderTicksOffsetEntry.setDouble(value)
            encoderTicksOffsetEntry.setPersistent()
        }


    val pitchAngle: Angle
        //      offset
        get() = (-162.0 + encoderRawTicks).degrees

    private var pitchSetpoint: Angle = pitchAngle
        set(value) {
            field = value.asDegrees.coerceIn(-50.0, 105.0).degrees
            println("Setpoint: $field")
        }
    val feedForward: Double
        get() = -0.04 * sin(pitchAngle)

    val pitchController = PDController(0.01, 0.001)

    var intakeState: IntakeState = IntakeState.INTAKING

    var prevIntakeState = intakeState

    init {
        pitchSetpoint = pitchAngle
        GlobalScope.launch {
            periodic {
                leftPitchAngleEntry.setDouble(pitchAngle.asDegrees)
                encoderTicksEntry.setDouble(encoderRawTicks)

                leftPitchMotor.setPercentOutput(feedForward + pitchController.update((pitchAngle - pitchSetpoint).asDegrees))

                if (fansOn) {
                    leftFans.speed = 1.0
//                    leftFans.setPercentOutput(100.0)
//                    leftFans.set(Relay.Value.kReverse)
                } else {
                    leftFans.speed = 0.0
//                    leftFans.setPercentOutput(0.0)
//                    leftFans.set(Relay.Value.kForward)
                }
//                println("Color Sensor : ${colorSensor.color}")
            }
        }
    }

    override suspend fun default()  {
        val t = Timer()
        periodic {

            if (intakeState != prevIntakeState) {
                t.start()
                prevIntakeState = intakeState
            }
            when (intakeState) {
                IntakeState.INTAKING -> {
                    balloonIntake()
                    animateToAngle(CARPET_ANGLE, t.get())
                    print("in IntakeState.Intaking")
                }
                IntakeState.DROPPING -> {
                    animateToAngle(TOTE_ANGLE, t.get())
                    if (pitchAngle <= 0.0.degrees) {
//                        balloonRelease()
                    }
                    println("In INtakeState.Dropping")
                }
                else -> {}
            }
        }
    }

    fun animateToAngle(angle: Angle, timeSinceStartSeconds: Double) {
        val startingAngle = pitchAngle.asDegrees
        var maxTime = (angle.asDegrees - startingAngle).absoluteValue / 180.0
        if (timeSinceStartSeconds <= maxTime) {
            pitchSetpoint = cubicMap(0.0, maxTime, startingAngle, angle.asDegrees, timeSinceStartSeconds).degrees
        }
    }



    fun balloonIntake() {
        leftAirValve.set(0.3)
    }

    fun balloonRelease() {
        leftAirValve.set(0.0)
    }
}

enum class IntakeState {
    INTAKING,
    DROPPING,
    MANUAL
}