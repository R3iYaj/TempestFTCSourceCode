package org.firstinspires.ftc.teamcode.DecodeChallenge.OpModes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DecodeChallenge.AllianceColor;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Systems.RobotMapping;
import org.firstinspires.ftc.teamcode.DecodeChallenge.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Systems.DecodeDriveSystemStateMachine;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Systems.FireSequenceSystemStateMachine;

@Autonomous(name="Autonomous Test", group="Test")
public class AutonomousTest extends LinearOpMode {
    private enum RobotState { Preloaded, Firing, MoveToFirstLine, MoveToSecondLine, MoveToThirdLine, Complete }

    private RobotMapping _robotMapping;
    private FireSequenceSystemStateMachine _fireSequenceSystem;
    private Follower _follower;
    private DecodeDriveSystemStateMachine _pathing;

    private final ElapsedTime _stateTimer = new ElapsedTime();
    private RobotState _currentAutoState = RobotState.Preloaded;

    private final AllianceColor allianceColor = AllianceColor.Blue;

    @Override
    public void runOpMode() {

        _robotMapping = new RobotMapping(hardwareMap);
        _follower = Constants.createFollower(hardwareMap);

        _pathing = new DecodeDriveSystemStateMachine(telemetry, _follower, _robotMapping, allianceColor);
        _fireSequenceSystem = new FireSequenceSystemStateMachine(telemetry, _robotMapping);

        telemetry.addData("OpMode", "Autonomous TEST");
        telemetry.update();

        waitForStart();

        FireSequenceSystemStateMachine.LaunchState launchState;

        while (opModeIsActive()) {

            _follower.update();

            launchState = _fireSequenceSystem.GetStatus();

            telemetry.addData("Auto State", _currentAutoState);
            telemetry.addData("Fire State", launchState);
            telemetry.addData("Pos X", _follower.getPose().getX());
            telemetry.addData("Pos Y", _follower.getPose().getY());

            switch (_currentAutoState){
                case Preloaded:
                    _fireSequenceSystem.InitFireMode();
                    _currentAutoState = RobotState.Firing;
                    break;

                case Firing:
                    if (launchState == FireSequenceSystemStateMachine.LaunchState.ReadyToFire){
                        _fireSequenceSystem.Fire();
                    }

                    if (launchState == FireSequenceSystemStateMachine.LaunchState.Off){
                        switch (_pathing.IdentifyNextSpecimen())
                        {
                            case 1:
                                _currentAutoState = RobotState.MoveToFirstLine;
                                _pathing.GoToFirstRow();
                                break;

                            case 2:
                                _currentAutoState = RobotState.MoveToSecondLine;
                                break;

                            case 3:
                                _currentAutoState = RobotState.MoveToThirdLine;
                                break;

                            default:
                                _currentAutoState = RobotState.Complete;
                        }
                    }
                    break;

                case MoveToFirstLine:



                    if (!_follower.isBusy()){
                        _currentAutoState = RobotState.MoveToSecondLine;
                    }
                    break;

                case MoveToSecondLine:
                    if (!_follower.isBusy()){
                        _currentAutoState = RobotState.MoveToThirdLine;
                    }
                    break;

                case MoveToThirdLine:
                    if (!_follower.isBusy()){
                        _currentAutoState = RobotState.Complete;
                    }
                    break;

                case Complete:
                    // Do nothing
                    break;
            }

            telemetry.update();
        }
    }
}