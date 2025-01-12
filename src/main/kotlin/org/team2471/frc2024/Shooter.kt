package org.team2471.frc2024

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion.following.demoMode
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2024.Drive.isRedAlliance

object Shooter: Subsystem("Shooter") {

    // this creates a network table for the Shooter subsystem
    private val table = NetworkTableInstance.getDefault().getTable("Shooter")

    // these are entries in our table for the network table
    private val motorRpmBottomEntry = table.getEntry("RPM Bottom")
    private val motorRpmTopEntry = table.getEntry("RPM Top")

    const val MAXRPM = 5800.0


    // TMM LESSON COMMENT:  Note how MotorControllers are made below, and where it gets their CAN bus id from RobotMap.kt

    val shooterMotorBottom = MotorController(FalconID(Falcons.SHOOTER_BOTTOM, "Shooter/Top"))
    val shooterMotorTop = MotorController(FalconID(Falcons.SHOOTER_TOP, "Shooter/Bottom"))

    val motorRpmTop
        get() = shooterMotorTop.velocity

    val motorRpmBottom
        get() = shooterMotorBottom.velocity

    var rpmTopSetpoint = 0.0
    var rpmBottomSetpoint = 0.0

    val averageRpm
        get() = (motorRpmBottom + motorRpmTop) / 2.0

    val averageRpmSetpoint
        get() = (rpmTopSetpoint + rpmBottomSetpoint) / 2.0

    var manualShootState = false

    val pitchCurve = MotionCurve()
    val rpmCurve = MotionCurve()

    init {

        shooterMotorBottom.config {
            feedbackCoefficient = 53.0 * (400.0 / 350.0)
            currentLimit(30, 40, 1.0)
            coastMode()
            inverted(true)
            configSim(DCMotor.getKrakenX60Foc(1), 0.005)
        }

        shooterMotorTop.config {
            feedbackCoefficient = 53.0 * (400.0 / 350.0)
            currentLimit(30, 40, 1.0)
            coastMode()
            inverted(true)
            configSim(DCMotor.getKrakenX60Foc(1), 0.005)
        }


        GlobalScope.launch {
            periodic {
                motorRpmBottomEntry.setDouble(motorRpmBottom)
                motorRpmTopEntry.setDouble(motorRpmTop)

                setPower(OI.driveRightTrigger)
            }
        }
    }

    override suspend fun default() {
        periodic {
        }
    }

    override fun onDisable() {
    }

    fun setRpms(rpm: Double) {

    }

    fun setPower(power: Double) {
        // TMM LESSON COMMENT: call setPercentOutput here for both the top and bottom motors
        shooterMotorTop.setPercentOutput(power)
        shooterMotorBottom.setPercentOutput(power)
    }

    // TMM LESSON COMMENT: also go to OI to map this to buttons
}