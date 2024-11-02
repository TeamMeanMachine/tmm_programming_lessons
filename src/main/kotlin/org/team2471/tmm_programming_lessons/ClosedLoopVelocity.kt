package org.team2471.tmm_programming_lessons

import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.cubicMap
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import kotlin.math.absoluteValue


// setRawOffset( Angle ) - make a button which can zero the little white arm
// software limits low and high
// motion profiling - CubicMap - look at balloon grabber in this project - use a new function animateToAngle() and use it on the PositionA and B functions
// arbitrary feed forward - coming soon

object ClosedLoopVelocity : Subsystem("ClosedLoop") {
    // make a table and an entry for motor angle
    val table = NetworkTableInstance.getDefault().getTable(name)
    val testMotorVelocityEntry = table.getEntry("Motor Velocity")
    val testMotorVelocitySetpointEntry = table.getEntry("Motor Velocity Setpoint")

    // declare a motor here, and use a SparkMaxID and id 16 from the robot map (SIMPLE_MOTOR)
    val testMotor = MotorController(SparkMaxID(Sparks.SIMPLE_MOTOR, "Motor1"))

    // declare a val to return the motor's encoder position.  Use a get() function
    val motorVelocity: Double
        get() = testMotor.velocity

    var motorPower: Double = 0.0

    // declare a var to contain the setpoint for the position pid
    var velocitySetpoint: Double = motorVelocity
        set(value) {
            field = value.coerceIn(0.0, 10000.0)
            // tell the motor to go to position ??
//            testMotor.setVelocitySetpoint(field, feedForward(motorVelocity))
        }

    // declare a pd controller from meanlib to control the motor with software
    val velocityController = PDController(0.00001, 0.00001)

    override fun postEnable() {
        motorPower = 0.0
    }
    init {
        testMotor.config {
            // next problem:
            // use println or network tables to display the motor position from the encoder, then manually turn the motor to set the feedbackCoefficient below
            // the units for the feedback coefficient are (native units) / (ticks)
            feedbackCoefficient = 1.0
            // pid here ??
//            pid {
//                p(0.250)
////                d(100.0)
//            }
        }

        GlobalScope.launch {
            periodic {
                val error = velocitySetpoint - motorVelocity
                motorPower += velocityController.update(error)
                testMotor.setPercentOutput(motorPower)
                testMotorVelocityEntry.setDouble(motorVelocity)
                testMotorVelocitySetpointEntry.setDouble(velocitySetpoint)

            }
        }
    }

//    override suspend fun default() {
//        val error = angleSetpoint - motorAngle
//        motorSpin(positionController.update(error.asDegrees))
//    }

    fun feedForward(angle: Angle) = 0.0079 * angle.cos()

    suspend fun positionA() {
        velocitySetpoint = 2000.0
        println("2000 rpm")
    }

    suspend fun positionB() {
        velocitySetpoint = 4000.0
        println("4000 rpm")
    }
}
