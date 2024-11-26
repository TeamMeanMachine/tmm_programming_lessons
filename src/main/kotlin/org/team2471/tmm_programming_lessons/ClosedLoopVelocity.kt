/*----------------------------------------------------------------------------*/ /* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package org.team2471.tmm_programming_lessons

import com.revrobotics.*
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem


/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
object ClosedLoopVelocity : Subsystem("ClosedLoopVelocity") {
    const val deviceID: Int = Sparks.SIMPLE_MOTOR
    private val m_motor: CANSparkMax
    private val m_pidController: SparkPIDController
    private val m_encoder: RelativeEncoder
    var kP: Double = 0.0
    var kI: Double = 0.0
    var kD: Double = 0.0
    var kIz: Double = 0.0
    var kFF: Double = 0.0
    var kMaxOutput: Double = 0.0
    var kMinOutput: Double = 0.0
    var maxRPM: Double = 0.0
    var maxVel: Double = 0.0
    var minVel: Double = 0.0
    var maxAcc: Double = 0.0
    var allowedErr: Double = 0.0
    var setPointPercentage = 0.0


    init {


        // initialize motor
        m_motor = CANSparkMax(deviceID, CANSparkLowLevel.MotorType.kBrushless)

        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        m_motor.restoreFactoryDefaults()


        // initialze PID controller and encoder objects
        m_pidController = m_motor.pidController
        m_encoder = m_motor.encoder


        // PID coefficients
        kP = 5e-5
        kI = 1e-6
        kD = 0.0
        kIz = 0.0
        kFF = 0.000156
        kMaxOutput = 1.0
        kMinOutput = -1.0
        maxRPM = 5700.0


        // Smart Motion Coefficients
        maxVel = 2000.0 // rpm
        maxAcc = 1500.0


        // set PID coefficients
        m_pidController.setP(kP)
        m_pidController.setI(kI)
        m_pidController.setD(kD)
        m_pidController.setIZone(kIz)
        m_pidController.setFF(kFF)
        m_pidController.setOutputRange(kMinOutput, kMaxOutput)


        /**
         * Smart Motion coefficients are set on a SparkPIDController object
         *
         * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
         * the pid controller in Smart Motion mode
         * - setSmartMotionMinOutputVelocity() will put a lower bound in
         * RPM of the pid controller in Smart Motion mode
         * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
         * of the pid controller in Smart Motion mode
         * - setSmartMotionAllowedClosedLoopError() will set the max allowed
         * error for the pid controller in Smart Motion mode
         */
        val smartMotionSlot = 0
        m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot)
        m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot)
        m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot)
        m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot)


        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP)
        SmartDashboard.putNumber("I Gain", kI)
        SmartDashboard.putNumber("D Gain", kD)
        SmartDashboard.putNumber("I Zone", kIz)
        SmartDashboard.putNumber("Feed Forward", kFF)
        SmartDashboard.putNumber("Max Output", kMaxOutput)
        SmartDashboard.putNumber("Min Output", kMinOutput)


        // display Smart Motion coefficients
        SmartDashboard.putNumber("Max Velocity", maxVel)
        SmartDashboard.putNumber("Min Velocity", minVel)
        SmartDashboard.putNumber("Max Acceleration", maxAcc)
        SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr)
        SmartDashboard.putNumber("Set Position", 0.0)
        SmartDashboard.putNumber("Set Velocity", 0.0)


        // button to toggle between velocity and smart motion modes
        SmartDashboard.putBoolean("Mode", false)

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
                val maxV = SmartDashboard.getNumber("Max Velocity", 0.0)
                val minV = SmartDashboard.getNumber("Min Velocity", 0.0)
                val maxA = SmartDashboard.getNumber("Max Acceleration", 0.0)
                val allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0.0)

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
                if ((maxV != maxVel)) {
                    m_pidController!!.setSmartMotionMaxVelocity(maxV, 0)
                    maxVel = maxV
                }
                if ((minV != minVel)) {
                    m_pidController!!.setSmartMotionMinOutputVelocity(minV, 0)
                    minVel = minV
                }
                if ((maxA != maxAcc)) {
                    m_pidController!!.setSmartMotionMaxAccel(maxA, 0)
                    maxAcc = maxA
                }
                if ((allE != allowedErr)) {
                    m_pidController!!.setSmartMotionAllowedClosedLoopError(allE, 0)
                    allowedErr = allE
                }

                val setPoint: Double
                val processVariable: Double
                val mode = SmartDashboard.getBoolean("Mode", false)
                if (mode) {
                    setPoint = SmartDashboard.getNumber("Set Velocity", 0.0)
                    m_pidController.setReference(setPoint, CANSparkBase.ControlType.kVelocity)
                    processVariable = m_encoder!!.velocity
                } else {
                    setPoint = SmartDashboard.getNumber("Set Position", 0.0)
                    /**
                     * As with other PID modes, Smart Motion is set by calling the
                     * setReference method on an existing pid object and setting
                     * the control type to kSmartMotion
                     */
                    m_pidController.setReference(setPoint, CANSparkBase.ControlType.kSmartMotion)
                    processVariable = m_encoder!!.position
                }

                SmartDashboard.putNumber("SetPoint", setPoint)
                SmartDashboard.putNumber("Process Variable", processVariable)
                SmartDashboard.putNumber("Output", m_motor!!.appliedOutput)
            }

        }

    }
}
