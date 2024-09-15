package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "April Tag Test", group = "Auto")
public class AprilTest extends LinearOpMode
{
    protected DcMotorEx right_drive1;
    protected DcMotorEx right_drive2;
    protected DcMotorEx left_drive1;
    protected DcMotorEx left_drive2;
    protected void initMotors() {

        right_drive1 = hardwareMap.get(DcMotorEx.class, "right_drive1");
        right_drive2 = hardwareMap.get(DcMotorEx.class, "right_drive2");
        left_drive1 = hardwareMap.get(DcMotorEx.class, "left_drive1");
        left_drive2 = hardwareMap.get(DcMotorEx.class, "left_drive2");

        right_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive2.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    @Override
    public void runOpMode() {
        initMotors();
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                . setCameraResolution(new Size(640,480))
                .build();
       /* while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING);


        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(255);*/


        waitForStart();

        while (opModeIsActive()) {
            if(tagProcessor.getDetections().size() > 0)
            {

                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addLine(String.format("XYZ %6.2f %6.2f %6.2f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                telemetry.addData("bearing", tag.ftcPose.bearing);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("range", tag.ftcPose.range);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("elevation", tag.ftcPose.elevation);
                telemetry.update();



            }
        }

    }
    protected void fixBot(float TargetX, float TargetY, float TargetYaw, AprilTagDetection tag)
    {
        int PosNeg = (tag.ftcPose.bearing > 180) ? -1 : 1;
        double InitialX = tag.ftcPose.x;
        double horizontal;
        while (TargetX != tag.ftcPose.x)
        {
            horizontal = (tag.ftcPose.x/InitialX)*PosNeg;
            right_drive1.setPower(-horizontal);
            right_drive2.setPower(horizontal);
            left_drive1.setPower(horizontal);
            left_drive2.setPower(-horizontal);
        }
        PosNeg = (tag.ftcPose.yaw > 180) ? -1 : 1;
        double InitialYaw = tag.ftcPose.yaw;
        double pivot;
        while (TargetYaw != tag.ftcPose.yaw)
        {
            pivot = (tag.ftcPose.yaw/InitialYaw)*PosNeg;
            right_drive1.setPower(-pivot);
            right_drive2.setPower(-pivot);
            left_drive1.setPower(pivot);
            left_drive2.setPower(pivot);
        }
        PosNeg = (TargetY > tag.ftcPose.y) ? -1 : 1;
        double InitialY = tag.ftcPose.y;
        double vertical;
        while (TargetY != tag.ftcPose.y)
        {
            vertical = (tag.ftcPose.y/InitialY)*PosNeg;
            right_drive1.setPower(vertical);
            right_drive2.setPower(vertical);
            left_drive1.setPower(vertical);
            left_drive2.setPower(vertical);
        }

    }

}
