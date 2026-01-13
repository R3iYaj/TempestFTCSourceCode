package org.firstinspires.ftc.teamcode.DecodeChallenge.Systems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Controllers.DistanceSensorController;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Controllers.LaunchController;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Controllers.ScooperController;

public class FireSequenceSystemStateMachine {

    public enum LaunchState { Off, SpinningUp, ReadyToFire, ScoopUp, ScoopDown, Loading, BallSense }

    private final Telemetry _telemetry;
    private final LaunchController _launcher;
    private final ScooperController _scooper;
    private final IntakeController _intake;
    private final DistanceSensorController _distanceSensor;


    private LaunchState _currentState;
    private final ElapsedTime _stateTimer = new ElapsedTime();
    private int _ballsFired;
    private boolean _fireAway;

    public FireSequenceSystemStateMachine(Telemetry telemetry, RobotMapping rm) {
        _telemetry = telemetry;
        _intake = new IntakeController(rm.UpperLeftIntake, rm.UpperRightIntake, rm.LowerLeftIntake, rm.LowerRightIntake);
        _launcher = new LaunchController(telemetry, rm.Goat, 1500);
        _scooper = new ScooperController(rm.Scooper, 300);
        _distanceSensor = new DistanceSensorController(rm.ColorSensor);

        _fireAway = false;
    }

    public void InitFireMode(){
        _launcher.Start();
        _ballsFired = 0;
        ChangeState(LaunchState.SpinningUp);
    }

    public boolean IsReadyToFire(){
        return _currentState == LaunchState.ReadyToFire;
    }

    public void Fire(){
        _fireAway = true;
    }

    public void Reload(){
        _intake.Activate();
        ChangeState(LaunchState.Loading);
    }

    public boolean IsFireComplete(){
        return _currentState == LaunchState.Off;
    }

    public LaunchState GetState() {

        switch (_currentState) {

            case SpinningUp:
//                BYPASS: Can't read motor velocity, no encoder cable connected to do so.
//                boolean isAtSpeed = _launcher.IsAtFullSpeed();
//                _telemetry.addData("Fire Mode", "Is at speed: " + isAtSpeed);
//                if (isAtSpeed && _stateTimer.milliseconds() > 500){
//                    _intake.Deactivate();
//                    ChangeState(LaunchState.ReadyToFire);
//                }

                _telemetry.addData("Fire Mode", "Speed check bypass, no encoder cable");
                if (_stateTimer.milliseconds() > 1000){
                    _intake.Deactivate();
                    ChangeState(LaunchState.ReadyToFire);
                }
                break;

            case ReadyToFire:
                // TODO: A BALL MIGHT FALL OUT DURING INTAKE, HAVE A WAY TO TIMEOUT
                if (_fireAway && _distanceSensor.GetDistanceInch() <= 3) {
                    _ballsFired++;
                    _fireAway = false;

                    _telemetry.addData("Fire Mode", "Dispatching ball #" + _ballsFired);
                    _scooper.ScoopUp();
                    ChangeState(LaunchState.ScoopUp);
                }
                break;

            case ScoopUp:
                if (!_scooper.IsBusy()) {
                    _scooper.ScoopDown();
                    ChangeState(LaunchState.ScoopDown);
                }
                break;

            case ScoopDown:
                if (!_scooper.IsBusy()) {
                    _intake.Activate();
                    ChangeState(LaunchState.BallSense);
                }
                break;

            case BallSense:
                if (_ballsFired >= 3) {
                    _launcher.Stop();
                    _intake.Deactivate();
                    ChangeState(LaunchState.Off);
                }
                else if (_stateTimer.milliseconds() > 100 && _distanceSensor.GetDistanceInch() <= 3) {
                    ChangeState(LaunchState.ReadyToFire);
                }
                break;

            case Loading:
                // TODO: May need to adjust timer to allow more/less time to load balls.
                if (_stateTimer.milliseconds() > 3000 && _distanceSensor.GetDistanceInch() <= 3){
                    _intake.Deactivate();
                    ChangeState(LaunchState.ReadyToFire);
                }
        }

        return _currentState;
    }

    private void ChangeState(LaunchState newState){
        _currentState = newState;
        _stateTimer.reset();
    }
}
