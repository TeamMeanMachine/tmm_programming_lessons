package org.team2471.tmm_programming_lessons

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees


object ClosedLoopPosition : Subsystem("ClosedLoop") {
    // declare a motor here, and use a SparkMaxID and id 16 from the robot map (SIMPLE_MOTOR)
    val testMotor = MotorController(SparkMaxID(Sparks.SIMPLE_MOTOR, "ClosedLoopPosition/testMotor"))

    // Problem 2 objective is to control the motor by using its encoder and software functions from meanlib

    // add a val to store the angle of the encoder, see 2024 pivot for an example


    // add a var to store the motor angle setpoint


    // create a val of type PDController from meanlib


    // create a function here to set the motor power or run the motor, which takes a percent - call setPercentOutput()
    fun motorSpin(percentage: Double ) {
        testMotor.setPercentOutput(percentage)
    }

    init {
        GlobalScope.launch {
            periodic {
                // calculate the error between angleSetpoint and motorAngle and store it in a val

                // update the PDController and set the motor power with the result

            }
        }
    }

    // make two suspending functions to map to buttons in OI which set the angle setpoint to different targets and add a print to each one
    suspend fun positionA() {

    }

    suspend fun positionB() {

    }
}
