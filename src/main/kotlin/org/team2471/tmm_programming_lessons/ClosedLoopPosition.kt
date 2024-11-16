package org.team2471.tmm_programming_lessons

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.cubicMap
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.Timer
import kotlin.math.absoluteValue


object ClosedLoopPosition : Subsystem("ClosedLoop") {
    // declare a motor here, and use a SparkMaxID and id 16 from the robot map (SIMPLE_MOTOR)
    val testMotor = MotorController(SparkMaxID(Sparks.SIMPLE_MOTOR, "Moter1"))

    // declare a val to return the motor's encoder position.  Use a get() function
    val motorAngle: Angle
        get() = testMotor.position.degrees

    // declare a var to contain the setpoint for the position pid
    var angleSetpoint: Angle = motorAngle
        set(value) {
            field = value.asDegrees.coerceIn(0.0, 1000.0).degrees
            testMotor.setPositionSetpoint(field.asDegrees)
        }

    // declare a pd controller from meanlib to control the motor with software
//    val postitionController = PDController(0.01, 0.0)

    // create a function here to set the motor power or run the motor, which takes a percent - call setPercentOutput()
    fun motorSpin(percentage: Double) {
        testMotor.setPercentOutput(percentage)
    }

    init {
        testMotor.config {
            // next problem:
            // use println or network tables to display the motor position from the encoder, then manually turn the motor to set the feedbackCoefficient below
            // the units for the feedback coefficient are (desired units) / (revolutions)
            feedbackCoefficient = 360.0
            pid {
                p(0.1)
                d(0.0)
            }
        }

        GlobalScope.launch {
            periodic {
//                val error = angleSetpoint - motorAngle
//                motorSpin(postitionController.update(error.asDegrees))
                println("Motor position is ${testMotor.position}")

            }
        }
    }

//    override suspend fun default() {
//        val error = angleSetpoint - motorAngle
//        motorSpin(postitionController.update(error.asDegrees))
//    }

    suspend fun animateToAngle(angle: Angle) {
        GlobalScope.launch {
            val timer = Timer()
            timer.start()
            val startingAngle = motorAngle
            var maxTime = (angle - startingAngle).asDegrees.absoluteValue / 1
            periodic {
                val timeSinceStartSeconds = timer.get()
                angleSetpoint =
                    cubicMap(0.0, maxTime, startingAngle.asDegrees, angle.asDegrees, timeSinceStartSeconds).degrees
                if (timeSinceStartSeconds > maxTime)
                    stop()

//            println("setpoint: ${pitchSetpoint.asDegrees.round(3)}")
            }

        }
    }


    suspend fun positianA() {
        animateToAngle(0.0.degrees)
        println("0 degress")
    }

    suspend fun positianB() {
        animateToAngle(90.0.degrees)
        println("180 degrees")
    }
}
