/*
The MIT License (MIT)

Copyright © 2021 Checkmate Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the “Software”), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.*
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.DriveConstants
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.DriveConstants.MOTOR_VELO_PID
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.DriveConstants.RUN_USING_ENCODER
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers.TrackingWheelLocalizer
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.SuperTrajectorySequenceRunner
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequenceBuilder
import kotlin.math.abs


/**
 * Mecanum drive implementation to work with roadrunner
 *
 * @see org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain
 */
@Config
class LongSchlong(hardwareMap: HardwareMap) : MecanumDrive(
    DriveConstants.kV,
    DriveConstants.kA,
    DriveConstants.kStatic,
    DriveConstants.TRACK_WIDTH,
    DriveConstants.TRACK_WIDTH,
    LATERAL_MULTIPLIER
), AbstractSubsystem {
    override val tag = "LongSchlong"
    override val subsystems = SubsystemMap{ tag }

    private val trajectorySequenceRunner: SuperTrajectorySequenceRunner
    val leftFront = Motors.LEFT_FRONT.get(hardwareMap)
    val leftRear = Motors.LEFT_REAR.get(hardwareMap)
    val rightRear = Motors.RIGHT_REAR.get(hardwareMap)
    val rightFront = Motors.RIGHT_FRONT.get(hardwareMap)
    private val motors = listOf(leftFront, leftRear, rightRear, rightFront)
    private val batteryVoltageSensor: VoltageSensor

    fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, false, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
    }

    fun trajectoryBuilder(startPose: Pose2d, reversed: Boolean): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
    }

    fun trajectoryBuilder(startPose: Pose2d, startHeading: Double): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
    }

    fun trajectorySequenceBuilder(startPose: Pose2d): TrajectorySequenceBuilder {
        return TrajectorySequenceBuilder(
            startPose,
            VEL_CONSTRAINT, ACCEL_CONSTRAINT,
            DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL
        )
    }

    fun turnAsync(angle: Double) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(poseEstimate)
                .turn(angle)
                .build()
        )
    }

    fun turn(angle: Double) {
        turnAsync(angle)
        waitForIdle()
    }

    fun followTrajectoryAsync(trajectory: Trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(trajectory.start())
                .addTrajectory(trajectory)
                .build()
        )
    }

    fun followTrajectory(trajectory: Trajectory) {
        followTrajectoryAsync(trajectory)
        waitForIdle()
    }

    fun followTrajectorySequenceAsync(trajectorySequence: TrajectorySequence?) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence)
    }

    fun followTrajectorySequence(trajectorySequence: TrajectorySequence?) {
        followTrajectorySequenceAsync(trajectorySequence)
        waitForIdle()
    }

    val lastError: Pose2d?
        get() = trajectorySequenceRunner.lastPoseError

    override fun loop() {
        updatePoseEstimate()
        trajectorySequenceRunner.update(poseEstimate, poseVelocity, voltage)?.let {
            setDriveSignal(it)
        }
    }

    private fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy) loop()
    }

    fun cancelSequence() {
        trajectorySequenceRunner.cancelSequence()
    }

    val isBusy: Boolean
        get() = trajectorySequenceRunner.isBusy

    fun setMode(runMode: DcMotor.RunMode?) {
        for (motor in motors) {
            motor.mode = runMode
        }
    }

    private val voltage
        get() = batteryVoltageSensor.voltage

    fun setWeightedDrivePower(drivePower: Pose2d) {
        var vel = drivePower
        if ((abs(drivePower.x) + abs(drivePower.y) + abs(drivePower.heading)) > 1) {
            // re-normalize the powers according to the weights
            val denominator =
                VX_WEIGHT * abs(drivePower.x) + VY_WEIGHT * abs(drivePower.y) + OMEGA_WEIGHT * abs(
                    drivePower.heading
                )
            vel = Pose2d(
                VX_WEIGHT * drivePower.x,
                VY_WEIGHT * drivePower.y,
                OMEGA_WEIGHT * drivePower.heading
            ).div(denominator)
        }
        setDrivePower(vel)
    }

    fun setPIDFCoefficients(runMode: RunMode?, coefficients: PIDFCoefficients) {
        val compensatedCoefficients = PIDFCoefficients(
            coefficients.p, coefficients.i, coefficients.d,
            coefficients.f * 12 / batteryVoltageSensor.voltage
        )
        for (motor in motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients)
        }
    }

    override fun getWheelPositions(): List<Double> {
        val wheelPositions: MutableList<Double> = ArrayList()
        for (motor in motors) {
            wheelPositions.add(DriveConstants.encoderTicksToInches(motor.currentPosition.toDouble()))
        }
        return wheelPositions
    }

    override fun getWheelVelocities(): List<Double> {
        val wheelVelocities: MutableList<Double> = ArrayList()
        for (motor in motors) {
            wheelVelocities.add(DriveConstants.encoderTicksToInches(motor.velocity))
        }
        return wheelVelocities
    }

    override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double,
                                frontRight: Double) {
        leftFront.power = frontLeft
        leftRear.power = rearLeft
        rightRear.power = rearRight
        rightFront.power = frontRight
    }

    override val rawExternalHeading = 0.0

    override fun getExternalHeadingVelocity() = 0.0

    companion object {
        @JvmField var TRANSLATIONAL_PID = PIDCoefficients(8.0, 0.0, 1.0)
        @JvmField var HEADING_PID = PIDCoefficients(8.0, 0.0, 1.0)
        @JvmField var LATERAL_MULTIPLIER = 1.0
        @JvmField var VX_WEIGHT = 1.0
        @JvmField var VY_WEIGHT = 1.0
        @JvmField var OMEGA_WEIGHT = 1.0
        val VEL_CONSTRAINT
            get() = getVelocityConstraint(
                DriveConstants.MAX_VEL,
                DriveConstants.MAX_ANG_VEL,
                DriveConstants.TRACK_WIDTH
            )
        val ACCEL_CONSTRAINT
            get() = getAccelerationConstraint(DriveConstants.MAX_ACCEL)

        private fun getVelocityConstraint(
            maxVel: Double,
            maxAngularVel: Double,
            trackWidth: Double
        ): TrajectoryVelocityConstraint {
            return MinVelocityConstraint(
                listOf(
                    AngularVelocityConstraint(maxAngularVel),
                    MecanumVelocityConstraint(maxVel, trackWidth)
                )
            )
        }

        private fun getAccelerationConstraint(maxAccel: Double): TrajectoryAccelerationConstraint {
            return ProfileAccelerationConstraint(maxAccel)
        }
    }

    override fun generateTelemetry(telemetry: Telemetry) {
        poseEstimate.let {
            telemetry.addData("x", it.x)
            telemetry.addData("y", it.y)
            telemetry.addData("h", Math.toDegrees(it.heading))
        }

        poseVelocity?.let {
            telemetry.addData("vX", it.x)
            telemetry.addData("vY", it.y)
            telemetry.addData("vH", Math.toDegrees(it.heading))
        }

        telemetry.addData("LF", leftFront.velocity)
        telemetry.addData("RF", rightFront.velocity)
        telemetry.addData("LB", leftRear.velocity)
        telemetry.addData("RB", rightRear.velocity)
    }

    init {
        val follower: TrajectoryFollower = HolonomicPIDVAFollower(
            TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
            Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5
        )

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        }

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()

        for (motor in motors) {
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // DONE: if desired, use setLocalizer() to change the localization method
        localizer = TrackingWheelLocalizer(hardwareMap)

        trajectorySequenceRunner = SuperTrajectorySequenceRunner(follower, HEADING_PID)
    }
}