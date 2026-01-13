package org.firstinspires.ftc.teamcode.DecodeChallenge.Controllers;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DecodeChallenge.AllianceColor;
import org.firstinspires.ftc.teamcode.DecodeChallenge.AprilTagConstant;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CameraController {

    private final Telemetry _telemetry;
    private final WebcamName _camera;
    private final VisionPortal _visionPortal;
    private final AprilTagProcessor _aprilTag;

    private final double _cameraOffsetX = 8.75; // From center (front to back)
    private final double _cameraOffsetY = 0; // From center (left to right)

    private final AllianceColor _allianceColor;

    public CameraController(Telemetry telemetry, WebcamName camera, AllianceColor allianceColor){
        _telemetry = telemetry;
        _camera = camera;
        _allianceColor = allianceColor;

        _aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        _visionPortal = VisionPortal.easyCreateWithDefaults(_camera);
    }


    public int GetDecode(){

        List<AprilTagDetection> currDetection = _aprilTag.getDetections();
        for (AprilTagDetection detection : currDetection) {

            if (detection.metadata == null || !detection.metadata.name.contains("Obelisk")) {

                _telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                _telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detection.center.x, detection.center.y));

                continue;
            }

            if (detection.id == AprilTagConstant.GPP ||
                detection.id == AprilTagConstant.PGP ||
                detection.id == AprilTagConstant.PPG) {
                return detection.id;
            }
        }

        return -1;
    }

    public Pose GetRobotPose(){

        int desiredGoalId = 0;
        switch (_allianceColor)
        {
            case Blue:
                desiredGoalId = AprilTagConstant.BlueGoal;
                break;

            case Red:
                desiredGoalId = AprilTagConstant.RedGoal;
                break;
        }

        List<AprilTagDetection> currDetection = _aprilTag.getDetections();
        for (AprilTagDetection detection : currDetection) {

            if (detection.metadata == null ||
                detection.metadata.name.contains("Obelisk")) {

                _telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                _telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detection.center.x, detection.center.y));

                continue;
            }

            if (detection.id != desiredGoalId)
            { continue; }

            _telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (inch)",
                    detection.robotPose.getPosition().x,
                    detection.robotPose.getPosition().y,
                    detection.robotPose.getPosition().z));
            _telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (deg)",
                    detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                    detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                    detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

            double tagX = detection.metadata.fieldPosition.get(0);
            double tagY = detection.metadata.fieldPosition.get(1);

            double range = detection.ftcPose.range;
            double bearing = detection.ftcPose.bearing;
            double yaw = detection.ftcPose.yaw;

            double robotX = tagX - (range * Math.cos(Math.toRadians(bearing * yaw)));
            double robotY = tagY - (range * Math.sin(Math.toRadians(bearing + yaw)));

            robotX -= _cameraOffsetX;
            robotY -= _cameraOffsetY;

            return new Pose(robotX, robotY, Math.toRadians(yaw));
        }

        return null;
    }

    public void Stop(){
        _visionPortal.close();
    }
}
