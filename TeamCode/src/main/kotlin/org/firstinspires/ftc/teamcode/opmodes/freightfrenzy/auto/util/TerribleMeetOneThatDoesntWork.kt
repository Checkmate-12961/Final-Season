package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence

@Disabled
@Autonomous(name = "TerribleMeetOne", preselectTeleOp = "TeleOp")
@Config
class TerribleMeetOneThatDoesntWork : BaseOpMode() {
    lateinit var traj: TrajectorySequence

    override fun setup() {
        // Make an array of the poses so we can iterate over them

        //val poses = arrayOf(pose1, pose2)

        // Initialize a trajectory builder
        val trajBuilder = robot.longSchlong.trajectorySequenceBuilder(pose0.toPose2d())

        /**pose1.y = 1.0
        if (robot.colorCone.leftColor == ColorCone.ConeColor.RED) {
            pose1.x = -28.0
            pose1.h = 20.0
        }
        else if (robot.colorCone.leftColor == ColorCone.ConeColor.BLUE) {
            pose1.x = 40.0
            pose1.h = -20.0
        }
        else {
            pose1.x = 6.0
            pose1.h = -5.0
        }
        pose2.y = -50.0
        pose2.x = pose1.x
        pose2.h = 0.0**/

        pose1.y = -50.0
        pose1.x = 8.0
        // Add poses that have been set to the trajectory builder

        trajBuilder.splineTo(pose1.toVector2d(), pose1.hR)


        // Build the trajectory
        traj = trajBuilder.build()
    }

    override fun preRunLoop() {
        // Follow the trajectory
        robot.longSchlong.followTrajectorySequenceAsync(traj)
    }

    companion object {
        @JvmField var pose0: Pose = Pose()
        @JvmField var pose1: Pose = Pose()
    }

    class Pose(@JvmField var x: Double = 0.0,
               @JvmField var y: Double = 0.0,
               @JvmField var h: Double = 0.0) {

        /**
         * Convert to a Pose2d
         *
         * @return x, y, and h in radians as a Pose2d
         */
        fun toPose2d(): Pose2d {
            return Pose2d(x, y, hR)
        }

        /**
         * Convert to a Vector2d
         *
         * @return x and y in a Vector2d
         */
        fun toVector2d(): Vector2d {
            return Vector2d(x, y)
        }

        /**
         * Heading in radians
         */
        val hR: Double
            get() = h * kotlin.math.PI / 180.0
    }
}