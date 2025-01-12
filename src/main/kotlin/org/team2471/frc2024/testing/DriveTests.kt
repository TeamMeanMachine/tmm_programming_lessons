package org.team2471.frc2024.testing


import org.team2471.frc2024.Drive
import org.team2471.frc2024.OI
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion.following.drive
import org.team2471.frc.lib.motion.following.odometryReset
import org.team2471.frc.lib.motion.following.tuneDrivePositionController
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.util.Timer
import kotlin.math.absoluteValue

suspend fun Drive.steeringTests() = use(this) {
    println("Got into steeringTests. Hi")
    for (module in 0..3) {
        println("Got into first for. Hi.")
        for (quadrant in 0..4) {
            Drive.modules[module].angleSetpoint = (quadrant * 90.0).degrees
            delay(0.5)
            println("#$module: Setpoint: ${modules[module].angleSetpoint} Angle: ${modules[module].angle}")
        }
        delay(0.5)
    }
}

suspend fun Drive.steerFeedbackCoefficientTest() = use(this) {
    for (i in 0..10) {
        for (quadrant in 0..4) {
            modules[0].angleSetpoint = (quadrant * 90.0).degrees
            delay(0.25)
        }
        delay(0.5)
    }
}
//

suspend fun Drive.driveTests() = use(this) {

    for (i in 0..3) {
        modules[i].setDrivePower(0.5)
        delay(1.0)
        modules[i].setDrivePower(0.0)
        delay(0.2)
    }
}

suspend fun Drive.fullTest() = use(this) {
    periodic {
        Drive.drive(OI.driveTranslation, OI.driveRotation)
    }
}

suspend fun Drive.tuneDrivePositionController(){  // = use(this) {
    tuneDrivePositionController(OI.driverController)
}

suspend fun Drive.rampTest() = use(Drive) {
    Drive.odometryReset()
    val t = Timer()
    t.start()
    periodic {
        drive(
            Vector2(0.0, 1.0),
            0.0,
            false
        )
        if (t.get() > 2.0) {
            drive(
                Vector2(0.0, 0.0),
                0.0,
                false
            )
            stop()
        }
    }
    println("endPos: ${position.y}")
}

suspend fun Drive.aimFTest() = use(Drive) {
    var turn = 0.0
    var upPressed = false
    var downPressed = false
    periodic {
        if (OI.driverController.dPad == Controller.Direction.UP) {
            upPressed = true
        } else if (OI.driverController.dPad == Controller.Direction.DOWN) {
            downPressed = true
        }
        if (OI.driverController.dPad != Controller.Direction.UP && upPressed) {
            upPressed = false
            turn += 0.005
        }
        if (OI.driverController.dPad != Controller.Direction.DOWN && downPressed) {
            downPressed = false
            turn -= 0.005
        }
        println("turn $turn")
        drive(
            Vector2(0.0, 0.0),
            turn,
            false
        )
    }
}

suspend fun Drive.currentTest() = use(this) {
    var power = 0.0
    var upPressed = false
    var downPressed = false
    periodic {
        if (OI.driverController.dPad == Controller.Direction.UP) {
            upPressed = true
        } else if (OI.driverController.dPad == Controller.Direction.DOWN) {
            downPressed = true
        }
        if (OI.driverController.dPad != Controller.Direction.UP && upPressed) {
            upPressed = false
            power += 0.05
        }
        if (OI.driverController.dPad != Controller.Direction.DOWN && downPressed) {
            downPressed = false
            power -= 0.05
        }

        var currModule = modules[0] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        currModule = modules[1] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        currModule = modules[2] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        currModule = modules[3] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)

        println("current: ${round(currModule.driveCurrent, 2)}  power: $power")
    }
}

suspend fun Drive.velocityTest() = use(this) {
    val velocitySetPoint = Vector2(0.0, 0.0)
    val velocityHeading = 100.0
    val t = Timer()
    t.start()
    periodic {
        driveWithVelocity(velocitySetPoint, velocityHeading.degrees)
        if (t.get() > 30.0) {
            this.stop()
        }
    }
}

suspend fun Drive.rotateTest() = use(Drive, name = "Rotation Test") {
    var deltaHeading = 0.0.degrees
    var prevHeading = heading
    val wheelDeltas = arrayListOf(0.0.degrees, 0.0.degrees, 0.0.degrees, 0.0.degrees)
    val prevWheelRots = arrayListOf(modules[0].rawWheelRotation, modules[1].rawWheelRotation, modules[2].rawWheelRotation, modules[3].rawWheelRotation)
    var wheelDistsFromCenter = modules.map { it.modulePosition.length }
    Drive.drive(Vector2(0.0, 0.0), 0.1)
    periodic {

        println("deltaHeading: ${deltaHeading.asDegrees}")
        if (deltaHeading > 1440.0.degrees) stop()

        deltaHeading += (heading - prevHeading).absoluteValue()
        prevHeading = heading

//        println(modules[0].rawWheelRotation)

        for (i in 0..3) {
            wheelDeltas[i] += (modules[i].rawWheelRotation - prevWheelRots[i]).absoluteValue()
            prevWheelRots[i] = modules[i].rawWheelRotation
//            Logger.recordOutput("Drive/Wheel $i Radius", ((wheelDistsFromCenter[i].times((heading - prevHeading).absoluteValue().asDegrees) / ((modules[i].rawWheelRotation - prevWheelRots[i]).absoluteValue()).asDegrees) * modules[i].gearRatio).asInches)
        }
    }

    val wheelRadii = arrayListOf(0.0.feet, 0.0.feet, 0.0.feet, 0.0.feet)

    for (i in 0..3) {
        wheelRadii[i] = (wheelDistsFromCenter[i].times(deltaHeading.asDegrees) / wheelDeltas[i].asDegrees.absoluteValue) * modules[i].gearRatio
        modules[i].wheelDiameter = wheelRadii[i]
    }

    println("Done! test: ${modules[0].wheelDiameter}")
}


