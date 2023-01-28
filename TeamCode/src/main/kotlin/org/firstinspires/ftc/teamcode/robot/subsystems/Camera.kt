package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.robot.HardwareNames
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import org.firstinspires.ftc.teamcode.robot.subsystems.camera.SubsystemOpenCvPipeline
import org.opencv.core.Scalar
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

class Camera(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = this.javaClass.simpleName
    override val subsystems = SubsystemMap { tag }

    private val webcam: OpenCvCamera

    var isStreaming: Boolean? = null
        private set

    /**
     * Closes the camera after the op mode is over.
     */
    override fun cleanup() {
        if (isStreaming == true) {
            webcam.stopStreaming()
            isStreaming = false
        } else if (isStreaming == null) {
            isStreaming = false
        }
    }

    var pipeline: SubsystemOpenCvPipeline? = null
        set(value) {
            pipeline?.let(subsystems::unregister)
            field = value!!
            subsystems.register(value)
            webcam.setPipeline(value)
        }

    init {
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            HardwareNames.Cameras.WEBCAM.get(hardwareMap),
            hardwareMap.appContext.resources.getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.packageName
            )
        )

        // listens for when the camera is opened
        webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                if (isStreaming == null) {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
                    isStreaming = true
                    // Streams camera output to the FTCDashboard
                    FtcDashboard.getInstance().startCameraStream(webcam, 24.0)
                }
            }

            override fun onError(errorCode: Int) {}
        })
    }
}

val Scalar.value: DoubleArray; get() = this.`val`