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
package org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.PIDFCoefficients

/*
 * Constants shared between multiple drive types.
 *
 * Constants generated by LearnRoadRunner.com/drive-constants
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 *  fields may also be edited through the dashboard (connect to the robot's WiFi network and
 *  navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 *  adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
object DriveConstants {
    /*
     * These are motor constants that should be listed online for your motors.
     */
    @JvmField var TICKS_PER_REV = 537.7
    @JvmField var MAX_RPM = 312.0

    const val RUN_USING_ENCODER = true
    @JvmField val MOTOR_VELO_PID: PIDFCoefficients? = null // PIDFCoefficients(0.0, 0.0, 0.0, 14.376)

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    @JvmField var WHEEL_RADIUS = 1.97 // in
    @JvmField var GEAR_RATIO = 1.0 // output (wheel) speed / input (motor) speed
    @JvmField var TRACK_WIDTH = 12.58 // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    // TODO: tune for the robot
    @JvmField var kV = 0.01583
    @JvmField var kA = 0.00064
    @JvmField var kStatic = 0.001825

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling). All distance units are inches.
     */
    /*
     * Note from LearnRoadRunner.com:
     * The velocity and acceleration constraints were calculated based on the following equation:
     * ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.85
     * Resulting in 54.66855022514893 in/s.
     * This is only 85% of the theoretical maximum velocity of the bot, following the recommendation above.
     * This is capped at 85% because there are a number of variables that will prevent your bot from actually
     * reaching this maximum velocity: voltage dropping over the game, bot weight, general mechanical inefficiencies, etc.
     * However, you can push this higher yourself if you'd like. Perhaps raise it to 90-95% of the theoretically
     * max velocity. The theoretically maximum velocity is 64.31594144135168 in/s.
     * Just make sure that your bot can actually reach this maximum velocity. Path following will be detrimentally
     * affected if it is aiming for a velocity not actually possible.
     *
     * The maximum acceleration is somewhat arbitrary and it is recommended that you tweak this yourself based on
     * actual testing. Just set it at a reasonable value and keep increasing until your path following starts
     * to degrade. As of now, it simply mirrors the velocity results in 54.66855022514893 in/s/s
     *
     * Maximum Angular Velocity is calculated as: maximum velocity / (trackWidth / 2) * (180 / Math.PI)

     */
    // TODO: re-tune the robot
    @JvmField var MAX_VEL = 30.0
    @JvmField var MAX_ACCEL = 30.0
    @JvmField var MAX_ANG_VEL = Math.toRadians(180.0)
    @JvmField var MAX_ANG_ACCEL = Math.toRadians(30.0)


    @JvmStatic fun encoderTicksToInches(ticks: Double): Double {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
    }

    @JvmStatic fun rpmToVelocity(rpm: Double): Double {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0
    }

    @JvmStatic fun getMotorVelocityF(ticksPerSecond: Double): Double {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond
    }
}