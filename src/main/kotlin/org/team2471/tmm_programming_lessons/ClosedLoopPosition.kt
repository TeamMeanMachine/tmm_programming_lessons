package org.team2471.tmm_programming_lessons

import edu.wpi.first.networktables.NetworkTableInstance
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
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.degrees
import org.team2471.tmm_programming_lessons.Drive.Module
import org.team2471.tmm_programming_lessons.Drive.modules
import org.team2471.tmm_programming_lessons.Drive.motorAngle0Entry
import kotlin.math.absoluteValue


// setRawOffset( Angle ) - make a button which can zero the little white arm
// software limits low and high
// motion profiling - CubicMap - look at balloon grabber in this project - use a new function animateToAngle() and use it on the PositionA and B functions
// arbitrary feed forward - coming soon

object ClosedLoopPosition : Subsystem("ClosedLoop") {
    // make a table and an entry for motor angle
    val table = NetworkTableInstance.getDefault().getTable(name)
    val testMotorAngleEntry = table.getEntry("Motor Angle")
    val testMotorAngleSetpointEntry = table.getEntry("Motor Angle Setpoint")

    val angleList = arrayListOf<Double>()

    // declare a motor here, and use a SparkMaxID and id 16 from the robot map (SIMPLE_MOTOR)
    val testMotor = MotorController(SparkMaxID(Sparks.SIMPLE_MOTOR, "Motor1"))

    // declare a val to return the motor's encoder position.  Use a get() function
    val motorAngle: Angle
        get() = testMotor.position.degrees

    // declare a var to contain the setpoint for the position pid
    var angleSetpoint: Angle = motorAngle
        set(value) {
            field = value.asDegrees.coerceIn(-28.0, 208.0).degrees
            // tell the motor to go to position ??
            testMotor.setPositionSetpoint(field.asDegrees, feedForward(motorAngle))
        }

    suspend fun testFeedForward(increment: Double) {
        var power = 0.0
        periodic (0.04) {
            power += increment
            testMotor.setPercentOutput(power)
            println("power = $power, angle = $motorAngle")
            if (power > 0.1) {
                stop()
            }
        }
    }

    // declare a pd controller from meanlib to control the motor with software
//    val positionController = PDController(0.0006, 0.0)

    // create a function here to set the motor power or run the motor, which takes a percent - call setPercentOutput()
    fun motorSpin(percentage: Double ) {
        testMotor.setPercentOutput(feedForward(motorAngle) + percentage)
    }

    fun zeroTestMotor(value: Double = -28.0) {
        testMotor.setRawOffset(value)
        angleSetpoint = value.degrees
        println("ZERO")
    }

    suspend fun animateToAngle(angle: Angle) {
        var t = 0.0
        val startingAngle = motorAngle.asDegrees
        var maxTime = (angle.asDegrees - startingAngle).absoluteValue / 180.0
        println("maxTime $maxTime")
        GlobalScope.launch {
            periodic {
                angleSetpoint = cubicMap(0.0, maxTime, startingAngle, angle.asDegrees, t).degrees
                t += 0.02
                if (t > maxTime) {
                    stop()
                    angleSetpoint = angle
                    println("set angleSetpoint $angle")
                }
            }
        }
    }

    override fun preEnable() {
        zeroTestMotor()
    }

    init {

        testMotor.config {
            // next problem:
            // use println or network tables to display the motor position from the encoder, then manually turn the motor to set the feedbackCoefficient below
            // the units for the feedback coefficient are (native units) / (ticks)
            feedbackCoefficient = 360.0 / 1.0
            // pid here ??
            pid {
                p(0.250)
//                d(100.0)
            }
        }

        GlobalScope.launch {
            periodic {
//                val error = angleSetpoint - motorAngle
//                motorSpin(positionController.update(error.asDegrees))
                testMotorAngleEntry.setDouble(motorAngle.asDegrees)
                testMotorAngleSetpointEntry.setDouble(angleSetpoint.asDegrees)

            }
        }
    }

//    override suspend fun default() {
//        val error = angleSetpoint - motorAngle
//        motorSpin(positionController.update(error.asDegrees))
//    }

    fun feedForward(angle: Angle) = 0.0079 * angle.cos()

    suspend fun positionA() {
        animateToAngle(0.0.degrees)
        println("0 degrees")
    }

    suspend fun positionB() {
        animateToAngle(90.0.degrees)
        println("90 degrees")
    }
}
