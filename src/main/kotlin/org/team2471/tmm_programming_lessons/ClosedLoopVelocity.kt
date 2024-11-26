/*----------------------------------------------------------------------------*/ /* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package org.team2471.tmm_programming_lessons

import com.revrobotics.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import kotlin.math.PI

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
object ClosedLoopVelocity : Subsystem("ClosedLoopVelocity") {
    private var m_motor: CANSparkMax? = null
    private var m_encoder: RelativeEncoder? = null
    var kP: Double = 0.0
    var kI: Double = 0.0
    var kD: Double = 0.0
    var kIz: Double = 0.0
    var kFF: Double = 0.0
    var kMaxOutput: Double = 0.0
    var kMinOutput: Double = 0.0
    var maxRPM: Double = 0.0
    var m_pidController: SparkPIDController
    var setPointPercentage = 0.0

    init {

        // initialize motor
        m_motor = CANSparkMax(Sparks.SIMPLE_MOTOR, CANSparkLowLevel.MotorType.kBrushless)

        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        m_motor!!.restoreFactoryDefaults()

        /**
         * In order to use PID functionality for a controller, a SparkPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = m_motor!!.pidController

        // Encoder object created to display position values
        m_encoder = m_motor!!.encoder

        // PID coefficients
        kP = 6e-5
        kI = 0.0
        kD = 0.0
        kIz = 0.0
        kFF = 0.000015
        kMaxOutput = 1.0
        kMinOutput = -1.0
        maxRPM = 5700.0

        // set PID coefficients
        m_pidController.setP(kP)
        m_pidController.setI(kI)
        m_pidController.setD(kD)
        m_pidController.setIZone(kIz)
        m_pidController.setFF(kFF)
        m_pidController.setOutputRange(kMinOutput, kMaxOutput)

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP)
        SmartDashboard.putNumber("I Gain", kI)
        SmartDashboard.putNumber("D Gain", kD)
        SmartDashboard.putNumber("I Zone", kIz)
        SmartDashboard.putNumber("Feed Forward", kFF)
        SmartDashboard.putNumber("Max Output", kMaxOutput)
        SmartDashboard.putNumber("Min Output", kMinOutput)


        GlobalScope.launch {
            periodic {

                // read PID coefficients from SmartDashboard
                val p = SmartDashboard.getNumber("P Gain", 0.0)
                val i = SmartDashboard.getNumber("I Gain", 0.0)
                val d = SmartDashboard.getNumber("D Gain", 0.0)
                val iz = SmartDashboard.getNumber("I Zone", 0.0)
                val ff = SmartDashboard.getNumber("Feed Forward", 0.0)
                val max = SmartDashboard.getNumber("Max Output", 0.0)
                val min = SmartDashboard.getNumber("Min Output", 0.0)

                // if PID coefficients on SmartDashboard have changed, write new values to controller
                if ((p != kP)) {
                    m_pidController!!.setP(p)
                    kP = p
                }
                if ((i != kI)) {
                    m_pidController!!.setI(i)
                    kI = i
                }
                if ((d != kD)) {
                    m_pidController!!.setD(d)
                    kD = d
                }
                if ((iz != kIz)) {
                    m_pidController!!.setIZone(iz)
                    kIz = iz
                }
                if ((ff != kFF)) {
                    m_pidController!!.setFF(ff)
                    kFF = ff
                }
                if ((max != kMaxOutput) || (min != kMinOutput)) {
                    m_pidController!!.setOutputRange(min, max)
                    kMinOutput = min
                    kMaxOutput = max
                }

                /**
                 * PIDController objects are commanded to a set point using the
                 * SetReference() method.
                 *
                 * The first parameter is the value of the set point, whose units vary
                 * depending on the control type set in the second parameter.
                 *
                 * The second parameter is the control type can be set to one of four
                 * parameters:
                 * com.revrobotics.CANSparkMax.ControlType.kDutyCycle
                 * com.revrobotics.CANSparkMax.ControlType.kPosition
                 * com.revrobotics.CANSparkMax.ControlType.kVelocity
                 * com.revrobotics.CANSparkMax.ControlType.kVoltage
                 */
                setPointPercentage = OI.driveRightTrigger
                m_pidController.setReference(setPointPercentage * maxRPM * PI, CANSparkBase.ControlType.kVelocity)

                SmartDashboard.putNumber("SetPoint", setPointPercentage * maxRPM)
                SmartDashboard.putNumber("ProcessVariable", m_encoder!!.velocity)
            }

        }
    }
}