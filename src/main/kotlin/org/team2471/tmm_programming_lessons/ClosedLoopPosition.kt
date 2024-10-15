package org.team2471.tmm_programming_lessons

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees


object ClosedLoopPosition : Subsystem("ClosedLoop") {
    // declare a motor here, and use a SparkMaxID and id 16 from the robot map (SIMPLE_MOTOR)
    val testMotor = MotorController(SparkMaxID(Sparks.SIMPLE_MOTOR, "Moter1"))

    val motorAngle: Angle
        get() = testMotor.position.degrees

    var angleSetpoint: Angle = motorAngle
        set(value) {
            field = value.asDegrees.coerceIn(0.0, 42.0).degrees
        }

    val postitionController = PDController(0.01, 0.0)

    // create a function here to set the motor power or run the motor, which takes a percent - call setPercentOutput()
    fun motorSpin(percentage: Double ) {
        testMotor.setPercentOutput(percentage)
    }

    init {
        GlobalScope.launch {
            periodic {
                val error = angleSetpoint - motorAngle
                motorSpin(postitionController.update(error.asDegrees))
            }
        }
    }

//    override suspend fun default() {
//        val error = angleSetpoint - motorAngle
//        motorSpin(postitionController.update(error.asDegrees))
//    }

    suspend fun positianA() {
        angleSetpoint = 3.0.degrees
        println("3 degress")
    }

    suspend fun positianB() {
        angleSetpoint = 1.0.degrees
        println("1 degrees")
    }
}
