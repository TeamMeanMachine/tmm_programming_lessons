/*----------------------------------------------------------------------------*/ /* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package org.team2471.tmm_programming_lessons

import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.*
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import kotlin.math.abs
import edu.wpi.first.units.Units.*
import org.team2471.frc.lib.units.degrees


/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
object ClosedLoopVelocity : Subsystem("ClosedLoopVelocity") {
    private val m_fx = TalonFX(1, "canivore")
    private val m_mmReq = MotionMagicVoltage(0.0)
    private val m_joystick = XboxController(0)

    private var m_printCount = 0

    init {
        val cfg = TalonFXConfiguration()

        /* Configure gear ratio */
        val fdb = cfg.Feedback
        fdb.SensorToMechanismRatio = 12.8 // 12.8 rotor rotations per mechanism rotation

        /* Configure Motion Magic */
        val mm = cfg.MotionMagic
        mm.withMotionMagicCruiseVelocity(Units.RotationsPerSecond.of(5.0)) // 5 (mechanism) rotations per second cruise
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
            // Take approximately 0.1 seconds to reach max accel
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Units.Second).of(100))

        val slot0 = cfg.Slot0
        slot0.kS = 0.25 // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12 // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01 // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 60.0 // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0.0 // No output for integrated error
        slot0.kD = 0.5 // A velocity error of 1 rps results in 0.5 V output

        var status = StatusCode.StatusCodeNotInitialized
        for (i in 0..4) {
            status = m_fx.configurator.apply(cfg)
            if (status.isOK) break
        }
        if (!status.isOK) {
            println("Could not configure device. Error: $status")
        }


        GlobalScope.launch {
            periodic {
                if (++m_printCount >= 10) {
                    m_printCount = 0
                    println("Pos: " + m_fx.position)
                    println("Vel: " + m_fx.velocity)
                    println()
                }
                /* Deadband the joystick */
                var leftY = m_joystick.leftY
                if (abs(leftY) < 0.1) leftY = 0.0

                m_fx.setControl(m_mmReq.withPosition(leftY * 10).withSlot(0))
                if (m_joystick.bButton) {
                    m_fx.setPosition(Units.Rotations.of(1.0))
                }

            }
        }
    }
}
