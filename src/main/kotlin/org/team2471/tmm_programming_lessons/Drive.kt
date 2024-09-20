package org.team2471.tmm_programming_lessons

import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.control.PDConstantFController
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.*
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller
import org.team2471.frc.lib.math.*
import org.team2471.frc.lib.motion.following.*
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.motion_profiling.following.SwerveParameters
import org.team2471.frc.lib.units.*
import org.team2471.frc2024.gyro.Gyro
import kotlin.math.absoluteValue
import kotlin.math.min

@OptIn(DelicateCoroutinesApi::class)
object Drive : Subsystem("Drive"), SwerveDrive {
    val robotHalfWidth = (32.0/2.0).inches
    val table = NetworkTableInstance.getDefault().getTable(name)
    val navXGyroEntry = table.getEntry("NavX Gyro")
//    val limitingFactor : Double
//        get() = 1.0
    val odometer0Entry = table.getEntry("Odometer 0")
    val odometer1Entry = table.getEntry("Odometer 1")
    val odometer2Entry = table.getEntry("Odometer 2")
    val odometer3Entry = table.getEntry("Odometer 3")
    val absoluteAngle0Entry = table.getEntry("Analog Angle 0")
    val absoluteAngle1Entry = table.getEntry("Analog Angle 1")
    val absoluteAngle2Entry = table.getEntry("Analog Angle 2")
    val absoluteAngle3Entry = table.getEntry("Analog Angle 3")
    val motorAngle0Entry = table.getEntry("Motor Angle 0")
    val motorAngle1Entry = table.getEntry("Motor Angle 1")
    val motorAngle2Entry = table.getEntry("Motor Angle 2")
    val motorAngle3Entry = table.getEntry("Motor Angle 3")

//    val motorPower0Entry = table.getEntry("Motor Power 0")
//    val motorPower1Entry = table.getEntry("Motor Power 1")
//    val motorPower2Entry = table.getEntry("Motor Power 2")
//    val motorPower3Entry = table.getEntry("Motor Power 3")
    val useGyroEntry = table.getEntry("Use Gyro")
    val angleToNodeEntry = table.getEntry("Angle To Node")
    val demoBoundingBoxEntry = table.getEntry("Demo Bounding Box")
    val demoAprilLookingAtEntry = table.getEntry("Demo Look At Tags")

//    val advantageSwerveStatesEntry = table.getEntry("SwerveStates")
//    val advantageSwerveTargetsEntry = table.getEntry("SwerveTargets")
    val rateCurve = MotionCurve()
    val demoLimitEntry = table.getEntry("Demo Mode Drive Limit")
    val plannedPathEntry = table.getEntry("Planned Path")
    val actualRouteEntry = table.getEntry("Actual Route")



//    val fieldObject = Field2d()
//    val fieldDimensions = Vector2(26.9375.feet.asMeters,54.0.feet.asMeters)
//    val fieldCenterOffset = fieldDimensions/2.0
//    val stateArray : DoubleArray
//        get() {
//            val dblArray = DoubleArray(8)
//            var i = 0
//            for (mod in modules) {
//                dblArray[i] = mod.speed
//                i++
//                dblArray[i] = mod.angle.asDegrees
//                i++
//            }
//            return dblArray
//        }
//    val targetArray : DoubleArray
//        get() {
//            val dblArray = DoubleArray(8)
//            var i = 0
//            for (mod in modules) {
//                dblArray[i] = (mod as Module).power
//                i++
//                dblArray[i] = mod.angleSetpoint.asDegrees
//                i++
//            }
//            return dblArray
//        }

    /**
     * Coordinates of modules
     * **/
    override val modules: Array<SwerveDrive.Module> = arrayOf(
        Module(
            MotorController(FalconID(Falcons.LEFT_FRONT_DRIVE, "Drive/LFD")),
            MotorController(FalconID(Falcons.LEFT_FRONT_STEER, "Drive/LFS")),
            Vector2(-9.75, 9.75).inches,
            Preferences.getDouble("Angle Offset 0",-120.76).degrees,
            CANCoders.CANCODER_FRONTLEFT,
            odometer0Entry,
            0
        ),
        Module(
            MotorController(FalconID(Falcons.RIGHT_FRONT_DRIVE, "Drive/RFD")),
            MotorController(FalconID(Falcons.RIGHT_FRONT_STEER, "Drive/RFS")),
            Vector2(9.75, 9.75).inches,
            Preferences.getDouble("Angle Offset 1",-290.3).degrees,
            CANCoders.CANCODER_FRONTRIGHT,
            odometer1Entry,
            1
        ),
        Module(
            MotorController(FalconID(Falcons.RIGHT_REAR_DRIVE, "Drive/RBD")),
            MotorController(FalconID(Falcons.RIGHT_REAR_STEER, "Drive/RBS")),
            Vector2(9.75, -9.75).inches,
            Preferences.getDouble("Angle Offset 2",-159.25).degrees,
            CANCoders.CANCODER_REARRIGHT,
            odometer2Entry,
            2
        ),
        Module(
            MotorController(FalconID(Falcons.LEFT_REAR_DRIVE, "Drive/LBD")),
            MotorController(FalconID(Falcons.LEFT_REAR_STEER, "Drive/LBS")),
            Vector2(-9.75, -9.75).inches,
            Preferences.getDouble("Angle Offset 3",-126.38).degrees,
            CANCoders.CANCODER_REARLEFT,
            odometer3Entry,
            3
        )
    )

    val gyro = Gyro
    private var gyroOffset = 0.0.degrees

    override var heading: Angle
        get() = (gyroOffset + gyro.angle).wrap()
        set(value) {
//            gyro.reset()
            gyroOffset = -gyro.angle + value
        }

    override val headingRate: AngularVelocity
        get() = -gyro.rate.degrees.perSecond

    override var velocity = Vector2(0.0, 0.0)
    override var acceleration: Vector2 = Vector2(0.0, 0.0)
    override var deltaPos: Vector2L = Vector2(0.0, 0.0).inches
    override var position = Vector2(0.0, 0.0)
    override var robotPivot = Vector2(0.0, 0.0).inches
    override var headingSetpoint = 0.0.degrees

    var autoAim: Boolean = false
    var angleToNode: Angle = 0.0.degrees
    val demoBondingBox: Boolean
        get() = demoBoundingBoxEntry.getBoolean(false)
    var prevDemoBox: Boolean = demoBondingBox
    val demoAprilLookingAt: Boolean
        get() = demoAprilLookingAtEntry.getBoolean(false)

    override val parameters: SwerveParameters = SwerveParameters(
        gyroRateCorrection = 0.0,
        kpPosition = 0.32,
        kdPosition = 0.6,
        kPositionFeedForward = 0.05,
        kpHeading = 0.008,
        kdHeading = 0.01,
        kHeadingFeedForward = 0.001,
        kMoveWhileSpin = 53.0,
    )

    override val carpetFlow = Vector2(0.0, 1.0)
    override val kCarpet = 0.0234 // how much downstream and upstream carpet directions affect the distance, for no effect, use  0.0 (2.5% more distance downstream)
    override val kTread = 0.0 //.04 // how much of an effect treadWear has on distance (fully worn tread goes 4% less than full tread)  0.0 for no effect
    override val plannedPath: NetworkTableEntry = plannedPathEntry
    override val actualRoute: NetworkTableEntry = actualRouteEntry
    override var lastResetTime: Double = 0.0
    override val gyroConnected: Boolean
        get() = gyro.isConnected

    val autoPDController = PDConstantFController(0.015, 0.04, 0.05)
    val teleopPDController =  PDConstantFController(0.012, 0.09, 0.05)
    var aimPDController = teleopPDController

//    var prevPosition : Vector2 = Vector2(0.0, 0.0)
//    var chargeMode = false

    var maxTranslation = 1.0
        get() =  if (demoMode) min(field, demoSpeed) else field

    const val MAX_INTAKE_TRANSLATION = 0.5
    const val MAX_SCORE_TRANSLATION = 0.3

    val isHumanDriving
        get() = OI.driveTranslation.length != 0.0 || OI.driveRotation != 0.0

    init {
        println("drive init")
        initializeSteeringMotors()

        if (!demoBoundingBoxEntry.exists()) {
            demoBoundingBoxEntry.setBoolean(false)
            demoBoundingBoxEntry.setPersistent()
        }


        if (!demoAprilLookingAtEntry.exists()) {
            demoAprilLookingAtEntry.setBoolean(false)
        }
        GlobalScope.launch(MeanlibDispatcher) {
//            odometer0Entry.setPersistent()
//            odometer1Entry.setPersistent()
//            odometer2Entry.setPersistent()
//            odometer3Entry.setPersistent()
            println("in drive global scope")
            val headingEntry = table.getEntry("Heading")
            val xEntry = table.getEntry("X")
            val yEntry = table.getEntry("Y")
            val poseEntry = table.getEntry("advantageScopePose")

            SmartDashboard.setPersistent("Use Gyro")
            SmartDashboard.setPersistent("Gyro Type")

            if (!SmartDashboard.containsKey("DemoSpeed")) {
                println("DemoSpeed does not exist, setting it to 1.0")
                SmartDashboard.getEntry("DemoSpeed").setDouble(1.0)
                SmartDashboard.setPersistent("DemoSpeed")
            }
//            SmartDashboard.putData("Field", fieldObject)
//            SmartDashboard.setPersistent("Field")

            navXGyroEntry.setBoolean(true)
            rateCurve.setMarkBeginOrEndKeysToZeroSlope(false)
            rateCurve.storeValue(1.0, 2.0)  // distance, rate
            rateCurve.storeValue(8.0, 6.0)  // distance, rate

            demoLimitEntry.setDouble(1.0)
            actualRouteEntry.setDoubleArray(doubleArrayOf())
            plannedPathEntry.setString("")

            //val defaultXYPos = doubleArrayOf(0.0,0.0)

      //      val robotHalfWidthFeet = robotHalfWidth.asFeet

        //    val reducedField = Vector2(fieldCenterOffset.x.meters.asFeet - robotHalfWidthFeet, fieldCenterOffset.y.meters.asFeet - robotHalfWidthFeet)
//            lastPosition = Pose2d(position.x.feet.asMeters+fieldCenterOffset.x, position.y.feet.asMeters+fieldCenterOffset.y, -Rotation2d((heading-90.0.degrees).asRadians))

            println("in init just before periodic")
            periodic {
                val (x, y) = position
//                if (x.absoluteValue > reducedField.x || y.absoluteValue > reducedField.y ){
//                    println("Coercing x inside field dimensions")
//                    x = x.coerceIn(-reducedField.x, reducedField.x)
//                    y = y.coerceIn(-reducedField.y, reducedField.y)
////                    position = Vector2(x, y)
//                }
                xEntry.setDouble(x)
                yEntry.setDouble(y)
                headingEntry.setDouble(heading.asDegrees)
                val poseWPI = FieldManager.convertTMMtoWPI(x.feet, y.feet, heading)
                poseEntry.setDoubleArray(doubleArrayOf(poseWPI.x, poseWPI.y, poseWPI.rotation.degrees))


//                if (FieldManager.homeField) {
                    absoluteAngle0Entry.setDouble((modules[0] as Module).absoluteAngle.asDegrees)
                    absoluteAngle1Entry.setDouble((modules[1] as Module).absoluteAngle.asDegrees)
                    absoluteAngle2Entry.setDouble((modules[2] as Module).absoluteAngle.asDegrees)
                    absoluteAngle3Entry.setDouble((modules[3] as Module).absoluteAngle.asDegrees)
//
//                    motorPower0Entry.setDouble((modules[0] as Module).driveCurrent)
//                    motorPower1Entry.setDouble((modules[1] as Module).driveCurrent)
//                    motorPower2Entry.setDouble((modules[2] as Module).driveCurrent)
//                    motorPower3Entry.setDouble((modules[3] as Module).driveCurrent)
//
//                }
                motorAngle0Entry.setDouble((modules[0] as Module).angle.wrap().asDegrees)
                motorAngle1Entry.setDouble((modules[1] as Module).angle.wrap().asDegrees)
                motorAngle2Entry.setDouble((modules[2] as Module).angle.wrap().asDegrees)
                motorAngle3Entry.setDouble((modules[3] as Module).angle.wrap().asDegrees)
           //     for (moduleCount in 0..3) { //changed to modules.indices, untested
             //       val module = (modules[moduleCount] as Module)
            //    }
            }
        }
    }

    override fun preEnable() {
        //initializeSteeringMotors()
        odometer0Entry.setDouble(Preferences.getDouble("odometer 0",0.0))
        odometer1Entry.setDouble(Preferences.getDouble("odometer 1",0.0))
        odometer2Entry.setDouble(Preferences.getDouble("odometer 2",0.0))
        odometer3Entry.setDouble(Preferences.getDouble("odometer 3",0.0))
        println("prefs at enable=${Preferences.getDouble("odometer 0",0.0)}")
    }
//    override fun postEnable(){
//       // initializeSteeringMotors()
//        println("Initialized From Post Enable")
//    }
    override fun onDisable() {
        if (odometer0Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 0", odometer0Entry.getDouble(0.0))
        if (odometer1Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 1", odometer1Entry.getDouble(0.0))
        if (odometer2Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 2", odometer2Entry.getDouble(0.0))
        if (odometer3Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 3", odometer3Entry.getDouble(0.0))
        actualRoute.setDoubleArray(doubleArrayOf())
        plannedPath.setString("")
    }

    override fun resetOdom() {
//        PoseEstimator.zeroOffset()
       // MAPoseEstimator.resetPose(FieldManager.convertTMMtoWPI(pose.position.x.feet, pose.position.y.feet, pose.heading))
    }

    fun zeroGyro() {
        heading = if (FieldManager.isBlueAlliance) 180.0.degrees else 0.0.degrees
        println("zeroed heading to $heading  alliance blue? ${FieldManager.isBlueAlliance}")
        //PoseEstimator.zeroOffset()
       // MAPoseEstimator.resetPose(FieldManager.convertTMMtoWPI(pose.position.x.feet, pose.position.y.feet, pose.heading))
        //gyro.reset()
    }

    override suspend fun default() {
        periodic {
            var turn = 0.0
            if (OI.driveRotation.absoluteValue > 0.001) {
                turn = OI.driveRotation
            }
            if (!useGyroEntry.exists()) {
                useGyroEntry.setBoolean(true)
            }
            val useGyro2 = useGyroEntry.getBoolean(true) && !DriverStation.isAutonomous()

//            if (autoAim) {
//                var error = (angleToNode - heading).wrap()
////                if (error.asDegrees.absoluteValue > 90.0) error = (error - 180.0.degrees).wrap()
//                turn = aimPDController.update(error.asDegrees)
//            }
            if (FieldManager.homeField) {
                angleToNodeEntry.setDouble(angleToNode.asDegrees)
            }
//            if (demoBondingBox) {
//                if (!prevDemoBox){
//                    position = Vector2(0.0, 0.0)
//                }
//                val limit = demoLimitEntry.getDouble(1.0)
//                val goalPos = Vector2(
//                    linearMap(-1.0, 1.0, -limit, limit, OI.driveTranslation.x),
//                    linearMap(-1.0, 1.0, -limit, limit, OI.driveTranslation.y)
//                )
//                val posDiff = (goalPos - position)// / 30.0
//                println(posDiff)
//                drive(
//                    posDiff,
//                    turn * maxRotation,
//                    useGyro2,
//                    useGyro2
//                )
//            } else if (demoAprilLookingAt) {
//
//                val tags = AprilTag.getAimingTarget()
//                drive(
//                    OI.driveTranslation * maxTranslation,
//                    turn * maxRotation,
//                    useGyro2,
//                    useGyro2
//                )
//
//                }
//                catch (ex: Exception) {
//                    print(ex.message)
//                    tag = null
//                }
//                if (tags != null) {
//                    if (tags.first != null && tags.second != null) {
//                        for (tag in tags.first!!) {
//                            val tagYaw = tag.yaw.degrees + if (tags.second == AprilTag.camBack) -23.0.degrees else 23.0.degrees
//
//                            val aimTurn = tagYaw / 35.0
//
//                            println("turn: ${aimTurn.asDegrees}  heading: ${heading.asDegrees}   tagYaw: ${tagYaw}   camera ${tags.second?.name}")
//
//                            drive(
//                                OI.driveTranslation * maxTranslation,
//                                aimTurn.asDegrees,
//                                true
//                            )
//                        }
//                    } else {
//                        println("Tag is null")
//                    }
//                }
//            } else {
                drive(
                    OI.driveTranslation * maxTranslation,
                    turn,
                    useGyro2,
                    useGyro2
                )
//            }
            prevDemoBox = demoBondingBox
            //println("heading=$heading useGyro=$useGyro2")
        }
    }
    fun initializeSteeringMotors() {
        for (moduleCount in 0..3) { //changed to modules.indices, untested
            val module = (modules[moduleCount] as Module)
            module.turnMotor.setRawOffset(module.absoluteAngle.asDegrees)
            println("Module: $moduleCount analogAngle: ${module.absoluteAngle}")
        }
    }

    fun resetDriveMotors() {
        for (moduleCount in 0..3) {
            val module = (modules[moduleCount] as Module)
            module.driveMotor.restoreFactoryDefaults()
            println("For module $moduleCount, drive motor's factory defaults were restored.")
        }
    }

    fun resetSteeringMotors() {
        for (moduleCount in modules.indices) {
            val module = (modules[moduleCount] as Module)
            module.turnMotor.restoreFactoryDefaults()
            println("For module $moduleCount, turn motor's factory defaults were restored.")
        }
    }

    fun brakeMode() {
        for (moduleCount in modules.indices) {
            val module = (modules[moduleCount] as Module)
            module.driveMotor.brakeMode()
        }
    }

    fun coastMode() {
        for (element in modules) { //switched from element in 0..modules.size-1, untested
            val module = (element as Module)
            module.driveMotor.coastMode()
            module.turnMotor.coastMode()
        }
    }

    class Module(
        val driveMotor: MotorController,
        val turnMotor: MotorController,
        override val modulePosition: Vector2L,
        override var angleOffset: Angle,
        canCoderID: Int,
        private val odometerEntry: NetworkTableEntry,
        val index: Int
    ) : SwerveDrive.Module {
        companion object {
            private const val ANGLE_MAX = 983
            private const val ANGLE_MIN = 47

            private val P = 0.0075 //0.010
            private val D = 0.00075
        }

        override val angle: Angle
            get() = turnMotor.position.degrees

        val canCoder : CANcoder = CANcoder(canCoderID)

        val absoluteAngle: Angle
            get() {
                return ((-canCoder.absolutePosition.value * 360.0).degrees - angleOffset).wrap()
            }

        override val treadWear: Double
            get() = linearMap(0.0, 10000.0, 1.0, 0.96, odometer).coerceIn(0.96, 1.0)

        val driveCurrent: Double
            get() = driveMotor.current

        private val pdController = PDController(P, D)

        override val speed: Double
            get() = driveMotor.velocity
        override val acceleration: Double
            get() = driveMotor.acceleration

        val power: Double
            get() {
                return driveMotor.output
            }

        override val currDistance: Double
            get() = driveMotor.position

        override var prevDistance: Double = 0.0

        override var odometer: Double
            get() = odometerEntry.getDouble(0.0)
            set(value) { odometerEntry.setDouble(value) }
        override var prevAngle: Angle = angle

        override fun zeroEncoder() {
            driveMotor.position = 0.0
        }

        override var angleSetpoint: Angle = 0.0.degrees
            set(value) = turnMotor.setPositionSetpoint((angle + (value - angle).wrap()).asDegrees)

        override fun setDrivePower(power: Double) {
            driveMotor.setPercentOutput(power)
        }

        val error: Angle
            get() = turnMotor.closedLoopError.degrees

        init {
            println("Drive.module.init")
            turnMotor.config(20) {
                feedbackCoefficient = 360.0 / 21.428  // 21.451 for bunnybot with same gearing
                inverted(true)
//                setSensorPhase(false)
                coastMode()
                setRawOffsetConfig(absoluteAngle.asDegrees)
                pid {
                    p(0.02 * 1024.0)
//                    d(0.0000025)
                }
            }
            driveMotor.config {
                brakeMode()
                //                    wheel diam / 12 in per foot * pi / ticks / gear ratio
                feedbackCoefficient = 4.0 / 12.0 * Math.PI / 5.42 * (90.8/96.0)
                currentLimit(70, 75, 1)
                openLoopRamp(0.2)
            }

        }

        override fun driveWithDistance(angle: Angle, distance: Length) {
            driveMotor.setPositionSetpoint(distance.asFeet)
            val error = (angle - this.angle).wrap()
            pdController.update(error.asDegrees)
        }

        override fun stop() {
            driveMotor.stop()
        }

        fun setAngleOffset() {
            val canAngle = -canCoder.absolutePosition.value * 360.0
            Preferences.setDouble("Angle Offset $index", canAngle)
            angleOffset = canAngle.degrees
            println("Angle Offset $index = $canAngle")
        }
    }
    fun setAngleOffsets() {
        for (element in modules) {
            val module = (element as Module)
            module.setAngleOffset()
        }
        initializeSteeringMotors()
    }

    suspend fun calibrateRobotPosition() = use(Drive) {
        //position = Vector2(-11.5, if (FieldManager.isBlueAlliance) 21.25 else -21.25)
        position = Vector2(0.0,0.0)
        zeroGyro()
//        PoseEstimator.zeroOffset()
    }
}

fun Drive.abortPath(): Boolean {
    return isHumanDriving
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
            power += 0.001
        }
        if (OI.driverController.dPad != Controller.Direction.DOWN && downPressed) {
            downPressed = false
            power -= 0.001
        }
//        for (moduleCount in 0..3) {
//            val module = modules[moduleCount] as Drive.Module
//        }
//        println()
//        println("power: $power")
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
    //    val currModule2 = modules[3] as Drive.Module
      //  currModule2.driveMotor.setPercentOutput(power)
        //currModule2.turnMotor.setPositionSetpoint(0.0)
       // println("current: ${round(currModule.driveCurrent, 2)}  power: $power")

    //        drive(
//            Vector2(0.0, power),
//            0.0,
//            false
//        )
    }
}