package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.subsystems.Zelda;

/**
 * This is a simple routine to test turning capabilities.
 */

@SuppressWarnings("unused")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        Zelda zelda = new Zelda(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        zelda.turn(Math.toRadians(ANGLE));
    }
}
