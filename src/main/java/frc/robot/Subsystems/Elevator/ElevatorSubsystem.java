package frc.robot.Subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();

    public ElevatorSubsystem(ElevatorIO io){
        this.io = io;
    }

    public enum WantedState {
        LEVEL_2,
        LEVEL_1,
        GROUND,
        DRIVER_STATION,
        IDLE
    }

    public enum SystemState {
        LEVEL_2,
        LEVEL_1,
        GROUND,
        DRIVER_STATION,
        IDLE
    }

    WantedState wantedState = WantedState.IDLE;
    SystemState systemState = SystemState.IDLE;

    private void applyStates(){
        switch(systemState){
            case LEVEL_2:
              io.setPositionMeters(ElevatorConstants.levelTwoHeightMeters, 75, 50);
              break;
            case LEVEL_1:
              io.setPositionMeters(ElevatorConstants.levelOneHeightMeters, 75, 50);
              break;
            case GROUND:
              io.setPositionMeters(ElevatorConstants.groundHeightMeters, 75, 50);
              break;
            case DRIVER_STATION:
              io.setPositionMeters(ElevatorConstants.driverStationHeightMeters, 75, 50);
              break;
            case IDLE:
              io.stop();
              break;
            default:
              io.stop();
              break;
        }
    }

    private SystemState handleStateTransitions(){
        switch(wantedState){
            case LEVEL_2:
              return SystemState.LEVEL_2;
            case LEVEL_1:
              return SystemState.LEVEL_1;
            case GROUND:
              return SystemState.GROUND;
            case DRIVER_STATION:
              return SystemState.DRIVER_STATION;
            case IDLE:
              return SystemState.IDLE;
            default:
              return SystemState.IDLE;
        }
    }

    public void SetWantedState(WantedState wantedState) {
      if (this.wantedState == wantedState) return;
      this.wantedState = wantedState;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        systemState = handleStateTransitions();
        applyStates();
    }
}
