package org.firstinspires.ftc.teamcode.DecodeChallenge.Systems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DecodeChallenge.AllianceColor;
import org.firstinspires.ftc.teamcode.DecodeChallenge.AprilTagConstant;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Controllers.CameraController;

import java.util.ArrayList;
import java.util.List;

public class DecodeDriveSystemStateMachine {

    public enum DriveState { DecdingNextSpeciment, DriveToRow, WaitForLoadTrigger, LoadingSpecimens, LoadingComplete, DriveToLaunch, ReadyToFire, Idle }
    private final Telemetry _telemetry;
    private final Follower _follower;
    private final CameraController _camera;

    private PathChain _pathMoveToFirstRow;
    private PathChain _pathApproachFirstRowStack;
    private PathChain _pathReturnFirstRowToLaunch;

    private PathChain _pathMoveToSecondRow;
    private PathChain _pathApproachSecondRowStack;
    private PathChain _pathReturnSecondRowToLaunch;

    private PathChain _pathMoveToThirdRow;
    private PathChain _pathApproachThirdRowStack;
    private PathChain _pathReturnThirdRowToLaunch;

    private final AllianceColor _allianceColor;

    private List<Integer> _completedSpecimens = new ArrayList<>();
    private int _targetSpecimenId;
    private int _startingSpecimen;
    private DriveState _currentDriveState = DriveState.Idle;
    private boolean _loadTriggerPulled;
    private boolean _returnTriggerPulled;

    public DecodeDriveSystemStateMachine(Telemetry telemetry, Follower follower, RobotMapping rm, AllianceColor allianceColor) {
        _telemetry = telemetry;
        _follower = follower;
        _allianceColor = allianceColor;
        _camera = new CameraController(telemetry, rm.Webcam, allianceColor);

        _loadTriggerPulled = false;
        _returnTriggerPulled = false;
    }

    public void Init(){

        int startingSpecimen = _camera.GetDecode();
        if (startingSpecimen != -1){
            _startingSpecimen = startingSpecimen;
        }
        else{
            _startingSpecimen = AprilTagConstant.GPP;
        }

        // TODO: MAKE THE DEFAULT START POSITION IS EITHER RED OR BLUE SIDE---DONE?? isn't it already determined by color?
        Pose startPosition = null;
        switch (_allianceColor)
        {
            case Blue:
                // TODO: WHAT IS THE BLUE SIDE START POINT? DONE
                startPosition = new Pose(60.056, 9.019, Math.toRadians(90));
                break;

            case Red:
                // TODO: WHAT IS THE RED SIDE START POINT? DONE
                startPosition = new Pose(86.890, 8.609, Math.toRadians(90));
                break;
        }

        Pose cameraPosition = _camera.GetRobotPose();

        if (cameraPosition != null) {
            _follower.setStartingPose(cameraPosition);
        }
        else {
            _follower.setStartingPose(startPosition);
        }

        BuildPaths();
    }

    public void ScanNextSpecimen(){
        if (_currentDriveState == DriveState.ReadyToFire){
            MarkSpecimentComplete(_targetSpecimenId);
        }

        ChangeState(DriveState.DecdingNextSpeciment);
    }
    public void StartLoading(){
        _loadTriggerPulled = true;
    }

    public void ReturnToLaunch(){
        _returnTriggerPulled = true;
    }

    public boolean IsWaitingToLoad(){
        return _currentDriveState == DriveState.WaitForLoadTrigger;
    }

    public boolean IsLoadComplete(){
        return _currentDriveState == DriveState.LoadingComplete;
    }

    public boolean IsReadyToFire(){
        return _currentDriveState == DriveState.ReadyToFire;
    }

    // TODO: THIS WILL HAVE TO CHANGE BASED ON TEAM COLOR
    // WE COULD ABSTRACT THIS BUILD PATHS INTO SEPARATE CLASSES
    // SO WE CAN HAVE ONE FOR A BLUE AND RED AND PASS IN REFERENCE,
    // OR DO A SWITCH CASE THAT HAS BOTH COLOR PATHS IN HERE
    private void BuildPaths(){

        _pathMoveToFirstRow = _follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 8.000),
                                new Pose(62.814, 32.989),
                                new Pose(40.432, 35.385)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        _pathApproachFirstRowStack = _follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(40.432, 35.385),
                                new Pose(15.417, 35.417)))
                .setTangentHeadingInterpolation()
                .build();

        _pathReturnFirstRowToLaunch = _follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.417, 35.417),
                                new Pose(53.737, 35.704),
                                new Pose(56.000, 8.000)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // TODO: NEED COORDINATES UPDATED FOR SECOND ROW
        _pathMoveToSecondRow = _follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.417, 35.417),
                                new Pose(53.737, 35.704),
                                new Pose(56.000, 8.000)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // TODO: NEED COORDINATES UPDATED FOR SECOND ROW
        _pathApproachSecondRowStack = _follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.417, 35.417),
                                new Pose(53.737, 35.704),
                                new Pose(56.000, 8.000)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // TODO: NEED COORDINATES UPDATED FOR SECOND ROW
        _pathReturnSecondRowToLaunch = _follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.417, 35.417),
                                new Pose(53.737, 35.704),
                                new Pose(56.000, 8.000)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // TODO: NEED COORDINATES UPDATED FOR THIRD ROW
        _pathMoveToThirdRow = _follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.417, 35.417),
                                new Pose(53.737, 35.704),
                                new Pose(56.000, 8.000)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // TODO: NEED COORDINATES UPDATED FOR THIRD ROW
        _pathApproachThirdRowStack = _follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.417, 35.417),
                                new Pose(53.737, 35.704),
                                new Pose(56.000, 8.000)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // TODO: NEED COORDINATES UPDATED FOR THIRD ROW
        _pathReturnThirdRowToLaunch = _follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.417, 35.417),
                                new Pose(53.737, 35.704),
                                new Pose(56.000, 8.000)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    private void GoToFirstRow(){
        _follower.followPath(_pathMoveToFirstRow);
    }

    private void ApproachFirstRowForLoading(){
        _follower.followPath(_pathApproachFirstRowStack);
    }

    private void ReturnFirstRowToLaunch(){
        _follower.followPath(_pathReturnFirstRowToLaunch);
    }

    private void GoToSecondRow(){
        _follower.followPath(_pathMoveToSecondRow);
    }

    private void ApproachSecondRowForLoading(){
        _follower.followPath(_pathApproachSecondRowStack);
    }

    private void ReturnSecondRowToLaunch(){
        _follower.followPath(_pathReturnSecondRowToLaunch);
    }

    private void GoToThirdRow(){
        _follower.followPath(_pathMoveToThirdRow);
    }

    private void ApproachThirdRowForLoading(){
        _follower.followPath(_pathApproachThirdRowStack);
    }

    private void ReturnThirdRowToLaunch(){
        _follower.followPath(_pathReturnThirdRowToLaunch);
    }

    private int IdentifyNextSpecimen(){

        if (!_completedSpecimens.contains(_startingSpecimen)) {
            return _startingSpecimen;
        }

        if (!_completedSpecimens.contains(AprilTagConstant.GPP)){
            return AprilTagConstant.GPP;
        }

        if (!_completedSpecimens.contains(AprilTagConstant.PGP)){
            return AprilTagConstant.PGP;
        }

        if (!_completedSpecimens.contains(AprilTagConstant.PPG)){
            return AprilTagConstant.PPG;
        }

        return -1;
    }

    private void MarkSpecimentComplete(int specimentId){
        _completedSpecimens.add(specimentId);
    }

    private void ChangeState(DriveState newState){
        _currentDriveState = newState;
        // NOTE: If you need to do something when the state changes, do it here:
    }

    private void SetRowDestination(int targetSpecimenId){
        switch (targetSpecimenId){
            case AprilTagConstant.GPP:
                GoToFirstRow();
                break;

            case AprilTagConstant.PGP:
                GoToSecondRow();
                break;

            case AprilTagConstant.PPG:
                GoToThirdRow();
                break;

            default:
                ChangeState(DriveState.Idle);
        }
    }

    private void SetApproach(int targetSpecimenId){
        switch (targetSpecimenId){

            case AprilTagConstant.GPP:
                ApproachFirstRowForLoading();
                break;

            case AprilTagConstant.PGP:
                ApproachSecondRowForLoading();
                break;

            case AprilTagConstant.PPG:
                ApproachThirdRowForLoading();
                break;
        }
    }

    private void SetReturnRoute(int targetSpecimenId){
        switch (targetSpecimenId){

            case AprilTagConstant.GPP:
                ReturnFirstRowToLaunch();
                break;

            case AprilTagConstant.PGP:
                ReturnSecondRowToLaunch();
                break;

            case AprilTagConstant.PPG:
                ReturnThirdRowToLaunch();
                break;
        }
    }

    public DriveState GetState(){

        switch (_currentDriveState){

            case DecdingNextSpeciment:
                _targetSpecimenId = IdentifyNextSpecimen();
                SetRowDestination(_targetSpecimenId);
                ChangeState(DriveState.DriveToRow);
                break;

            case DriveToRow:
                if(!_follower.isBusy()){
                    ChangeState(DriveState.WaitForLoadTrigger);
                }
                break;

            case WaitForLoadTrigger:

                if (_loadTriggerPulled){
                    SetApproach(_targetSpecimenId);
                    ChangeState(DriveState.LoadingSpecimens);
                    _loadTriggerPulled = false;
                }
                break;

            case LoadingSpecimens:
                if (!_follower.isBusy()){
                    ChangeState(DriveState.LoadingComplete);
                }
                break;

            case LoadingComplete:

                if (_returnTriggerPulled){
                    SetReturnRoute(_targetSpecimenId);
                    ChangeState(DriveState.DriveToLaunch);
                }

            case DriveToLaunch:
                if (!_follower.isBusy()){
                    ChangeState(DriveState.ReadyToFire);
                }
                break;
        }

        return _currentDriveState;
    }
}