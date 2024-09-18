package org.team2471.tmm_programming_lessons.testing

import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.motion.following.driveAlongPath
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.units.*
import org.team2471.tmm_programming_lessons.*
import kotlin.math.absoluteValue
import kotlin.math.absoluteValue

suspend fun pathFollowTest() = use(Drive) {
    Drive.heading = -180.0.degrees
    val newPath = Path2D("Path Follow Test")
    println("position: ${Drive.position}, ${Drive.combinedPosition}")
    val p1currentPose = PoseEstimator.currentPose

    newPath.addVector2(p1currentPose)
    newPath.addVector2(Vector2(p1currentPose.x, p1currentPose.y - 10.0))

    var time = 2.5
    //val finalHeading = if (FieldManager.isBlueAlliance) 180.0 else 0.0
    val currentHeading = Drive.heading
    print(currentHeading)

    newPath.addEasePoint(0.0,0.0)
    newPath.addEasePoint(time, 1.0)

    newPath.addHeadingPoint(0.0, currentHeading.asDegrees)
    newPath.addHeadingPoint(time, 170.0)
    Drive.driveAlongPath(newPath)
    print(Drive.heading)
}

suspend fun autoFeedForwardTest(motor: MotorController, motorAngle: () -> Angle, angleList: ArrayList<Double>, powerInterval: Double = 0.01) {
    val powerList: ArrayList<Double> = ArrayList()

    for (angle in angleList) {
        var power = 0.0
        var lastAngleError: Double? = null
        periodic {
            val angleError = angle - motorAngle.invoke().asDegrees
            val rate = (lastAngleError ?: angleError) - angleError
            if (angleError.absoluteValue < 0.5 && rate.absoluteValue < 0.5) {
                powerList.add(power.round(2))
                this.stop()
            } else if (angleError > angle) {
                power += powerInterval
                motor.setPercentOutput(power)
            } else if (angleError < angle) {
                power -= powerInterval
                motor.setPercentOutput(power)
            }
            lastAngleError = angleError
        }
    }
    println("Angles: ${angleList}")
    println("Powers: ${powerList}")
}
