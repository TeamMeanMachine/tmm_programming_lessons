package org.team2471.tmm_programming_lessons

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem



object OpenLoopSubsystem : Subsystem("OpenLoop") {
    // declare a motor here, and use a SparkMaxID and id 11 from the robot map (SIMPLE_MOTOR)

    val motor = MotorController(SparkMaxID(Sparks.SIMPLE_MOTOR, "OpenLoop/motor"))

    // create a function here to set the motor power or run the motor, which takes a percent - call setPercentOutput()
    /**
     * @param power: double between -1 and 1, 0 is off
     */
    fun runMotor(power: Double) {
        motor.setPercentOutput(power)
        println("power: $power")
    }

    init {
        GlobalScope.launch {
            periodic {
                runMotor(OI.driveRightTrigger - OI.driveLeftTrigger)
            }
        }
    }
//    override suspend fun default() {
//        periodic {
//            runMotor(OI.driveRightTrigger - OI.driveLeftTrigger)
//        }
//    }
}