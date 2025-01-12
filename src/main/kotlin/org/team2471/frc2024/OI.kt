package org.team2471.frc2024

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.input.*
import org.team2471.frc.lib.math.*
import org.team2471.frc.lib.motion.following.demoMode
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2024.AprilTag.resetCameras
import org.team2471.frc2024.Drive.isBlueAlliance
import org.team2471.frc2024.Shooter.shooterMotorBottom
import org.team2471.frc2024.Shooter.shooterMotorTop
//import org.team2471.frc2024.Shooter.manualShootState
import kotlin.math.absoluteValue

object OI : Subsystem("OI") {
    val driverController = XboxController(0)
    val operatorController = XboxController(1)

    private val deadBandDriver = 0.08
    private val deadBandOperator = 0.1

    private val driveTranslationX: Double
        get() = driverController.leftThumbstickX.deadband(deadBandDriver).squareWithSign()

    private val driveTranslationY: Double
        get() = -driverController.leftThumbstickY.deadband(deadBandDriver).squareWithSign()

    val driveTranslation: Vector2
        get() = if (isBlueAlliance) Vector2(driveTranslationX, driveTranslationY) else -Vector2(driveTranslationX, driveTranslationY)  //does owen want this cubed?

    val driveRotation: Double
        get() = (driverController.rightThumbstickX.deadband(deadBandDriver)).cube() // * 0.6

    val driveLeftTrigger: Double
        get() = driverController.leftTrigger

    val driveLeftTriggerFullPress: Boolean
        get() = driverController.leftTriggerFullPress

    val driveRightTrigger: Double
        get() = driverController.rightTrigger

    val operatorLeftTrigger: Double
        get() = operatorController.leftTrigger

    val operatorLeftY: Double
        get() = operatorController.leftThumbstickY.deadband(0.2)

    val operatorLeftX: Double
        get() = operatorController.leftThumbstickX.deadband(0.2)

    val operatorRightTrigger: Double
        get() = operatorController.rightTrigger

    val operatorRightX: Double
        get() = operatorController.rightThumbstickX.deadband(0.2)

    val operatorRightY: Double
        get() = operatorController.rightThumbstickY.deadband(0.0)

    val opX: Boolean
        get() = operatorController::x.get()

    init {
        driverController::back.whenTrue {
            Drive.zeroGyro()
            Drive.initializeSteeringMotors() //not needed 02/05
        }

        // TMM LESSON COMMENT: add two mappings from a button to rotating and from a second button to stop the motor
        driverController::b.whenTrue {
            Shooter.setPower(0.5)
        }
        driverController::a.whenTrue {
            Shooter.setPower(0.0)
        }


        GlobalScope.launch {
            periodic {
                // Driver Rumble
//                if (Robot.isTeleopEnabled && (Shooter.motorRpmTop - Shooter.rpmTopSetpoint).absoluteValue + (Shooter.motorRpmBottom - Shooter.rpmBottomSetpoint).absoluteValue < 500.0 && Shooter.rpmTopSetpoint + Shooter.rpmBottomSetpoint > 20.0) {
//                    driverController.rumble = 1.0
//                } else if (Robot.isTeleopEnabled && (Intake.intakeMotorTop.output > 0.0 || Intake.intakeMotorBottom.output > 0.0)) {
//                    driverController.rumble = 0.7
//                } else {
//                    driverController.rumble = 0.0
//                }

//                println("driveRotation $driveRotation driveTranslation ${driveTranslation}")

                // Operator Rumble
//                if (Shooter.manualShootState && Robot.isTeleopEnabled) {
//                    operatorController.rumble = 1.0
//                } else {
//                    operatorController.rumble = 0.0
//                }
            }
        }
    }
}
