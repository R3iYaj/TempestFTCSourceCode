package org.firstinspires.ftc.teamcode.DecodeChallenge.OpModes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DecodeChallenge.AllianceColor;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Systems.RobotMapping;
import org.firstinspires.ftc.teamcode.DecodeChallenge.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Systems.DecodeDriveSystemStateMachine;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Systems.FireSequenceSystemStateMachine;

@Autonomous(name="Autonomous Test", group="Test")
public class AutonomousTest extends LinearOpMode {
    private enum RobotState { Preloaded, Firing, ReadyToLoad, LoadComplete, ReadyToFire, OperationComplete }

    private RobotMapping _robotMapping;
    private IntakeController _intake;
    private FireSequenceSystemStateMachine _fireSequenceSystem;
    private Follower _follower;
    private DecodeDriveSystemStateMachine _driveSystem;

    private final ElapsedTime _stateTimer = new ElapsedTime();
    private RobotState _currentRobotState;

    private final AllianceColor allianceColor = AllianceColor.Blue;
    private int _runLoops = 0;

    @Override
    public void runOpMode() {

        _robotMapping = new RobotMapping(hardwareMap);
        _follower = Constants.createFollower(hardwareMap);

        _intake = new IntakeController(_robotMapping.UpperLeftIntake, _robotMapping.UpperRightIntake, _robotMapping.LowerLeftIntake, _robotMapping.LowerRightIntake);
        _fireSequenceSystem = new FireSequenceSystemStateMachine(telemetry, _robotMapping);
        _driveSystem = new DecodeDriveSystemStateMachine(telemetry, _follower, _robotMapping, allianceColor);
        _driveSystem.Init();

        telemetry.addData("OpMode", "Autonomous TEST");
        telemetry.update();

        _currentRobotState = RobotState.Preloaded;

        waitForStart();

        while (opModeIsActive()) {

            _follower.update(); // Required to keep pedro pathing data updated

            telemetry.addData("Auto State", _currentRobotState);
            telemetry.addData("Fire State", _fireSequenceSystem.GetState());
            telemetry.addData("Drive State", _driveSystem.GetState());
            telemetry.addData("Pos X", _follower.getPose().getX());
            telemetry.addData("Pos Y", _follower.getPose().getY());

            switch (_currentRobotState){

                case Preloaded:
                    _fireSequenceSystem.InitFireMode();
                    ChangeState(RobotState.Firing);
                    break;

                case Firing:
                    if (_fireSequenceSystem.IsReadyToFire()){
                        _fireSequenceSystem.Fire();
                    }

                    if (_fireSequenceSystem.IsFireComplete()){
                        _driveSystem.ScanNextSpecimen();
                        ChangeState(RobotState.ReadyToLoad);
                    }
                    break;

                case ReadyToLoad:
                    if (_driveSystem.IsWaitingToLoad()){
                        _intake.Activate();
                        _driveSystem.StartLoading();
                        ChangeState(RobotState.LoadComplete);
                    }
                    break;

                case LoadComplete:
                    if (_driveSystem.IsLoadComplete()){
                        _intake.Deactivate();
                        _driveSystem.ReturnToLaunch();
                        ChangeState(RobotState.ReadyToFire);
                    }
                    break;

                case ReadyToFire:
                    if (_runLoops >= 3){
                        ChangeState(RobotState.OperationComplete);
                        break;
                    }

                    if(_driveSystem.IsReadyToFire()){
                       ChangeState(RobotState.Preloaded);
                    }
                    break;

                case OperationComplete:
                    telemetry.addData("OpMode", "COMPLETE!");
                    break;
            }

            telemetry.update();
        }
    }

    private void ChangeState(RobotState newState){
        _currentRobotState = newState;
        // NOTE: If you need to do something when the state changes, do it here:
    }
}