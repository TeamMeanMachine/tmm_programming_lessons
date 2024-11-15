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
    val pEntry = table.getEntry("Motor P")
    val feedForwardEnrty = table.getEntry("Motor Feed Forward")
    val arbitraryFeedForwardEntry = table.getEntry("Motor Arbitrary Feed Forward")

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
            // tell the motor to go to velocity ??
            testMotor.setVelocitySetpoint(field, arbitraryFeedForwardEntry.getDouble(3.0))
            //println("Feed forward: ${feedForward(field)}")
        }

//    fun feedForward(rpm: Double) = 1024.0 * rpm / 5925.0

//    // declare a pd controller from meanlib to control the motor with software
//    val velocityController = PDController(0.00001, 0.00001)

    override fun postEnable() {
        motorPower = 0.0
    }

    init {
        pEntry.setDouble(0.0000001)
        feedForwardEnrty.setDouble(0.0)
        arbitraryFeedForwardEntry.setDouble(0.0)

        testMotor.config {
            // the units for the feedback coefficient are (native units) / (ticks)
            feedbackCoefficient = 1.0
            // pid here ??
            pid {
                p(pEntry.getDouble(0.00001))
//                d(0.00001)
            }

        }


        GlobalScope.launch {
            periodic {
//                val error = velocitySetpoint - motorVelocity
//                motorPower += velocityController.update(error)
//                testMotor.setPercentOutput(motorPower)
                testMotorVelocityEntry.setDouble(motorVelocity)

                velocitySetpoint = testMotorVelocitySetpointEntry.getDouble(0.01)
                testMotor.setP(pEntry.getDouble(0.00001))
//                testMotor.setF
            }
        }
    }

//    override suspend fun default() {
//        val error = angleSetpoint - motorAngle
//        motorSpin(positionController.update(error.asDegrees))
//    }

//    fun feedForward(angle: Angle) = 0.0079 * angle.cos()

    suspend fun positionA() {
        velocitySetpoint = 2000.0
        println("2000 rpm")
    }

    suspend fun positionB() {
        velocitySetpoint = 4000.0
        println("4000 rpm")
    }
}
