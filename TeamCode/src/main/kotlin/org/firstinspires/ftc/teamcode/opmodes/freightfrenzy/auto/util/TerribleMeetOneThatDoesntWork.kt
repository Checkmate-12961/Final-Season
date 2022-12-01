package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.ColorCone
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence

@Disabled
@Autonomous(name = "TerribleMeetOne", preselectTeleOp = "TeleOp")
@Config
class TerribleMeetOneThatDoesntWork : BaseOpMode() {
    lateinit var traj: TrajectorySequence

    override fun setup() {
        // Make an array of the poses so we can iterate over them
        val poses = arrayOf(pose1, pose2, poseIntake, pose3, poseIntake, pose4, pose5)

        // Dimensions for blue left
        pose0.x = 31.5
        pose0.y = 7.5

        // pose1 scores low
        pose1.x = 48.0
        pose1.y = 13.0
        pose1.h = 0.0

        pose2.x = 36.0
        pose2.y = 50.0

        poseIntake.x = 10.0
        poseIntake.y = 60.0
        poseIntake.h = Math.toRadians(90.0)

        // pose3 scores high across blue-left/red-right wall
        pose3.x = 62.0
        pose3.y = 48.0
        pose3.h = 90.0

        // Add intake exe
        // pose4 scores high across blue wall
        pose4.x = 48.0
        pose4.y = 62.0
        pose4.h = 270.0

        // pose5 avoids obstacles
        pose5.x = 60.0
        pose5.y = 60.0
        pose5.h = 270.0

        // pose*colour* parks for vision
        poseRed.x = 12.0
        poseRed.y = 60.0

        poseGreen.x = 36.0
        poseGreen.y = 60.0

        poseBlue.x = 59.0
        poseBlue.y = 60.0

        //val poses = arrayOf(pose1, pose2)

        // Initialize a trajectory builder
        val trajBuilder = robot.longSchlong.trajectorySequenceBuilder(pose0.toPose2d())


        for (pose in poses) {
            if (pose.x != 0.0 || pose.y != 0.0 || pose.h != 0.0) {
                trajBuilder.splineTo(pose.toVector2d(), pose.hR)
            } else break
        }
        pose1.y = 1.0
        if (robot.colorCone.leftColor == ColorCone.ConeColor.RED) {
            trajBuilder.splineTo(poseRed.toVector2d(), poseRed.hR)
        }
        else if (robot.colorCone.leftColor == ColorCone.ConeColor.GREEN) {
            trajBuilder.splineTo(poseGreen.toVector2d(), poseGreen.hR)
        }
        else {
            trajBuilder.splineTo(poseBlue.toVector2d(), poseBlue.hR)
        }
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
        @JvmField var poseIntake: Pose = Pose()
        @JvmField var poseRed: Pose = Pose()
        @JvmField var poseGreen: Pose = Pose()
        @JvmField var poseBlue: Pose = Pose()
        @JvmField var pose1: Pose = Pose()
        @JvmField var pose2: Pose = Pose()
        @JvmField var pose3: Pose = Pose()
        @JvmField var pose4: Pose = Pose()
        @JvmField var pose5: Pose = Pose()
        @JvmField var pose6: Pose = Pose()
        @JvmField var pose7: Pose = Pose()
        @JvmField var pose8: Pose = Pose()
        @JvmField var pose9: Pose = Pose()
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