package org.firstinspires.ftc.teamcode.DecodeChallenge.Systems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Controllers.DistanceSensorController;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Controllers.LaunchController;
import org.firstinspires.ftc.teamcode.DecodeChallenge.Controllers.ScooperController;

public class FireSequence {

    public enum LaunchState { Off, SpinningUp, ReadyToFire, ScoopUp, ScoopDown, Loading, BallSense }
    public  int _maxBalls = 3;
    private final Telemetry _telemetry;
    private final LaunchController _launcher;
    private final ScooperController _scooper;
    private final IntakeController _intake;
    private final DistanceSensorController _distanceSensor;


    private static final double _launcherTargetVelocity = 2600; // NOTE: Observed max speed: 2600
    private LaunchState _currentState;
    private final ElapsedTime _stateTimer = new ElapsedTime();
    private int _ballsFired;
    private boolean _fireAway;

    public FireSequence(Telemetry telemetry, RobotMapping rc) {
        _telemetry = telemetry;
        _intake = new IntakeController(rc.UpperLeftIntake, rc.UpperRightIntake, rc.LowerLeftIntake, rc.LowerRightIntake);
        _launcher = new LaunchController(rc.Goat, _launcherTargetVelocity);
        _scooper = new ScooperController(rc.Scooper);
        _distanceSensor = new DistanceSensorController(rc.ColorSensor);

        _fireAway = false;
    }

    public void InitFireMode(){
        _launcher.StartVelocity();
        _ballsFired = 0;
        ChangeState(LaunchState.SpinningUp);
    }

    public void Fire(){
        _fireAway = true;
    }

    public void Reload(){
        _intake.Activate();
        ChangeState(LaunchState.Loading);
    }

    public LaunchState GetStatus() {

        _launcher.ReportVelocity(_telemetry);

        switch (_currentState) {

            case SpinningUp:
                // BYPASS: Can't read motor velocity, no encoder cable connected to do so.
                if (_launcher.IsAtFullSpeed() || _stateTimer.milliseconds() > 1000){
                    _intake.Deactivate();
                    ChangeState(LaunchState.ReadyToFire);
                }
                break;

            case ReadyToFire:
                if ((_fireAway && _distanceSensor.GetDistanceInch() <= 3) || _stateTimer.milliseconds() > 2000) {
                    _ballsFired++;
                    _fireAway = false;

                    _telemetry.addData("Fire Mode", "Dispatching ball #" + _ballsFired);
                    _scooper.ScoopUp(300);
                    ChangeState(LaunchState.ScoopUp);
                }
                break;

            case ScoopUp:
                if (_scooper.IsActuationComplete() || _stateTimer.milliseconds() > 2000) {
                    _scooper.ScoopDown(800);
                    ChangeState(LaunchState.ScoopDown);
                }
                break;

            case ScoopDown:
                if (_scooper.IsActuationComplete() || _stateTimer.milliseconds() > 2000) {
                    _intake.Activate();
                    ChangeState(LaunchState.BallSense);
                }
                break;

            case BallSense:
                if (_ballsFired >= _maxBalls) {
                    _launcher.Stop();
                    _intake.Deactivate();
                    ChangeState(LaunchState.Off);
                }
                else if (_distanceSensor.GetDistanceInch() <= 3 || _stateTimer.milliseconds() > 2000) {
                    _intake.Deactivate();
                    ChangeState(LaunchState.ReadyToFire);
                }
                break;

            case Loading:
                // TODO: May need to adjust timer to allow more/less time to load balls. og = 3000
                if (_stateTimer.milliseconds() > 2000 && _distanceSensor.GetDistanceInch() <= 3){
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
