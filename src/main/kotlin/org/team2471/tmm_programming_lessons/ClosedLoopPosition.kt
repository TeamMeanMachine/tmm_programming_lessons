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
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.degrees
import org.team2471.tmm_programming_lessons.Drive.Module
import org.team2471.tmm_programming_lessons.Drive.modules
import org.team2471.tmm_programming_lessons.Drive.motorAngle0Entry


// setRawOffset( Angle ) - make a button which can zero the little white arm
// software limits low and high
// motion profiling - CubicMap - look at balloon grabber in this project - use a new function animateToAngle() and use it on the PositionA and B functions
// arbitrary feed forward - coming soon

object ClosedLoopPosition : Subsystem("ClosedLoop") {
    // make a table and an entry for motor angle
    val table = NetworkTableInstance.getDefault().getTable(name)
    val testMotorAngleEntry = table.getEntry("Motor Angle")


    // declare a motor here, and use a SparkMaxID and id 16 from the robot map (SIMPLE_MOTOR)
    val testMotor = MotorController(SparkMaxID(Sparks.SIMPLE_MOTOR, "Motor1"))

    // declare a val to return the motor's encoder position.  Use a get() function
    val motorAngle: Angle
        get() = testMotor.position.degrees

    // declare a var to contain the setpoint for the position pid
    var angleSetpoint: Angle = motorAngle
        set(value) {
            field = value.asDegrees.coerceIn(-1000.0, 1000.0).degrees
            // tell the motor to go to position ??
            testMotor.setPositionSetpoint(field.asDegrees)
        }

    // declare a pd controller from meanlib to control the motor with software
//    val positionController = PDController(0.0006, 0.0)

    // create a function here to set the motor power or run the motor, which takes a percent - call setPercentOutput()
    fun motorSpin(percentage: Double ) {
        testMotor.setPercentOutput(percentage)
    }

    fun zero() {

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

            }
        }
    }

//    override suspend fun default() {
//        val error = angleSetpoint - motorAngle
//        motorSpin(positionController.update(error.asDegrees))
//    }

    suspend fun positionA() {
        angleSetpoint = 45.0.degrees
        println("45 degrees")
    }

    suspend fun positionB() {
        angleSetpoint = 90.0.degrees
        println("90 degrees")
    }
}
