package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.subsystems.Zelda;

/*
 * This is a simple routine to test translational drive capabilities.
 */

@SuppressWarnings("unused")
public class StrafeTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @SuppressWarnings("StatementWithEmptyBody")
    @Override
    public void runOpMode() throws InterruptedException {
        Zelda zelda = new Zelda(hardwareMap);

        Trajectory trajectory = zelda.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        zelda.followTrajectory(trajectory);

        Pose2d poseEstimate = zelda.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
