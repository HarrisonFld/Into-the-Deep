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
    AprilTagProcessor tagProcessor;
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
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                . setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .build();
       /* while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING);


        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(255);*/


        waitForStart();
        boolean start = false;
        while (opModeIsActive()) {
            if(tagProcessor.getDetections().size() > 0)
            {

                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                if(!start && tag != null && tagProcessor != null)
                {
                    start = true;
                    fixBot(1.03f, 22, -4);
                }
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
    protected void fixBot(float TargetX, float TargetY, float TargetYaw)
    {
        AprilTagDetection tag = tagProcessor.getDetections().get(0);
        int PosNeg;
        double pivot;
        while (Math.abs(TargetYaw-tag.ftcPose.yaw) > 1)
        {
            if(tagProcessor.getDetections().size() > 0) {
                tag = tagProcessor.getDetections().get(0);
                PosNeg = (tag.ftcPose.yaw < 0) ? 1 : -1;
                pivot = (Math.abs(Math.abs(tag.ftcPose.yaw)-Math.abs(TargetYaw))/90)*PosNeg;
                right_drive1.setPower(-pivot);
                right_drive2.setPower(-pivot);
                left_drive1.setPower(pivot);
                left_drive2.setPower(pivot);
            }
        }
        tag = tagProcessor.getDetections().get(0);
        PosNeg = (tag.ftcPose.x > 0) ? 1 : -1;
        double InitialX = tag.ftcPose.x;
        double horizontal;
        while (Math.abs(Math.abs(TargetX)-Math.abs(tag.ftcPose.x)) > 0.2)
        {
            if(tagProcessor.getDetections().size() > 0) {
                tag = tagProcessor.getDetections().get(0);
                horizontal = (Math.abs(tag.ftcPose.x / InitialX) * PosNeg) * 0.6;
                right_drive1.setPower(-horizontal);
                right_drive2.setPower(horizontal);
                left_drive1.setPower(horizontal);
                left_drive2.setPower(-horizontal);
            }
        }
        tag = tagProcessor.getDetections().get(0);
        PosNeg = (TargetY > tag.ftcPose.y) ? -1 : 1;
        double InitialY = tag.ftcPose.y;
        double vertical;
        while (Math.abs(TargetY-tag.ftcPose.y) > 0.2)
        {
            if(tagProcessor.getDetections().size() > 0) {
                tag = tagProcessor.getDetections().get(0);
                vertical = ((tag.ftcPose.y / InitialY) * PosNeg) * 0.6;
                right_drive1.setPower(vertical);
                right_drive2.setPower(vertical);
                left_drive1.setPower(vertical);
                left_drive2.setPower(vertical);
            }
        }
        right_drive1.setPower(0);
        right_drive2.setPower(0);
        left_drive1.setPower(0);
        left_drive2.setPower(0);
    }

}
