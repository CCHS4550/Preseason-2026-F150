package frc.robot.Subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private ElevatorIoInputsAutologged inputs = new ElevatorIoInputsAutologged();

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
              io.setPosition(ElevatorConstants.levelTwoHeightMeters);
              break;
            case LEVEL_1:
              io.setPosition(ElevatorConstants.levelOneHeightMeters);
              break;
            case GROUND:
              io.setPosition(ElevatorConstants.groundHeightMeters);
              break;
            case DRIVER_STATION:
              io.setPosition(ElevatorConstants.driverStationHeightMeters);
              break;
            case IDLE:
              io.stopElevator();
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

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        systemState = handleStateTransitions();
        applyStates();
    }
}
