package org.team2471.tmm_programming_lessons

import edu.wpi.first.wpilibj.Timer
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import kotlin.math.absoluteValue

data class Pose(val wristPosition: Vector2, val wristAngle: Angle, val pivotAngle: Angle) {
    companion object {
        val current: Pose
            get() = Pose(Arm.wristPosition, Intake.wristAngle, Intake.pivotAngle)
        val START_POSE = Pose(Vector2(0.0, 9.0), -90.0.degrees, -90.0.degrees)
        val GROUND_INTAKE_FRONT = Pose(Vector2(18.0, 10.0), 90.0.degrees, -90.0.degrees)
        val GROUND_INTAKE_POSE_NEAR = Pose(Vector2(18.0, 11.0), 90.0.degrees, 0.0.degrees)
        val GROUND_INTAKE_POSE_FAR = Pose(Vector2(40.0, 11.0), 90.0.degrees, 0.0.degrees)
        val SHELF_INTAKE_POSE = Pose(Vector2(0.0, 9.0), -90.0.degrees, 180.0.degrees)
        val LOW_SCORE = Pose(Vector2(0.0, 9.0), -90.0.degrees, 180.0.degrees)
        val BACK_MIDDLE_SCORE_CONE_AWAY = Pose(Vector2(-35.0, 28.0), -180.0.degrees, -180.0.degrees)
        val BACK_MIDDLE_SCORE_CONE_TOWARD = Pose(Vector2(-34.0, 26.0), -120.0.degrees, 0.0.degrees)
        val HIGH_SCORE = Pose(Vector2(0.0, 9.0), -90.0.degrees, 180.0.degrees)
        val FRONT_DRIVE_POSE = Pose(Vector2(0.0, 9.0), 92.0.degrees, -90.0.degrees)
        val BACK_DRIVE_POSE = Pose(Vector2(0.0, 9.0), -92.0.degrees, -90.0.degrees)
        val FLIP_INTAKE_TO_BACK_POSE = Pose(Vector2(-28.0, 26.0), 90.0.degrees, -90.0.degrees)
        val FLIP_INTAKE_TO_BACK_WRIST = Pose(Vector2(-28.0, 26.0), -90.0.degrees, -90.0.degrees)
        val FLIP_INTAKE_TO_FRONT_POSE = Pose(Vector2(28.0, 20.0), -90.0.degrees, -90.0.degrees)
        val FLIP_INTAKE_TO_FRONT_WRIST = Pose(Vector2(28.0, 20.0), 90.0.degrees, -90.0.degrees)
    }

    operator fun plus(otherPose: Pose) = Pose(wristPosition + otherPose.wristPosition, wristAngle + otherPose.wristAngle, pivotAngle + otherPose.pivotAngle)
}

suspend fun animateToPose(pose: Pose) = use(Arm, Intake) {
    println("Starting Animation $pose")
    val path = Path2D("newPath")
    path.addVector2(Pose.current.wristPosition)
    path.addVector2(pose.wristPosition)
    var distance = path.length
    var rate = 30.0  //  inches per second
    var wristPosTime = distance / rate

    distance = (Intake.wristSetpoint.asDegrees - Intake.wristAngle.asDegrees).absoluteValue
    rate = 40.0 // deg per second
    var wristTime = distance / rate

    distance = (Intake.pivotSetpoint.asDegrees - Intake.pivotAngle.asDegrees).absoluteValue
    rate = 30.0 // deg per second
    var pivotTime = distance / rate

    val time = maxOf(wristPosTime, wristTime, pivotTime)
    println("wristPosT: ${round(wristPosTime, 1)}    wristT: ${round(wristTime, 1)}      pivotT: ${round(pivotTime, 1)}")

    path.addEasePoint(0.0,0.0)
    path.addEasePoint(time, 1.0)

    val wristCurve = MotionCurve()
    wristCurve.storeValue(0.0, Pose.current.wristAngle.asDegrees)
    wristCurve.storeValue(time, pose.wristAngle.asDegrees)

    val pivotCurve = MotionCurve()
    pivotCurve.storeValue(0.0, Pose.current.pivotAngle.asDegrees)
    pivotCurve.storeValue(time, pose.pivotAngle.asDegrees)

    val timer = Timer()
    timer.start()
    periodic {
        val t = timer.get()
        Arm.wristPosition = path.getPosition(t)
        Intake.wristSetpoint = wristCurve.getValue(t).degrees
        Intake.pivotSetpoint = pivotCurve.getValue(t).degrees
//        if (pose == Pose.FLIP_INTAKE_TO_BACK || pose == Pose.FLIP_INTAKE_TO_FRONT) println("t: ${round(t, 1)}  actualWrist: ${round(Intake.wristAngle.asDegrees, 1)} wristSetpoint: ${round(Intake.wristSetpoint.asDegrees, 1)}    actualWristPos: ${Arm.forwardKinematics(Arm.shoulderMotor.position.degrees, Arm.elbowAngle)}   wristPos: ${Arm.wristPosition}")
        if (t > time) {
            this.stop()
        }
    }
}
