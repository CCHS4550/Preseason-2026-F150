package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotConstants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public final Trigger L2;
    public final Trigger DS;


    public Elevator(ElevatorIO io){
        this.io = io;
        L2 = new Trigger(() -> 
          (io.getPositionMeters() - ElevatorConstants.levelTwoHeightMeters) > -ElevatorConstants.positionError && 
          (io.getPositionMeters() - ElevatorConstants.levelTwoHeightMeters) < ElevatorConstants.positionError);
        DS = new Trigger(() -> 
          (io.getPositionMeters() - ElevatorConstants.driverStationHeightMeters) > -ElevatorConstants.positionError && 
          (io.getPositionMeters() - ElevatorConstants.driverStationHeightMeters) < ElevatorConstants.positionError);

    }

    public enum WantedState {
        LEVEL_2,
        LEVEL_1,
        GROUND,
        DRIVER_STATION,
        IDLE,
        OPENLOOP
      }

    public enum SystemState {
        LEVEL_2,
        LEVEL_1,
        GROUND,
        DRIVER_STATION,
        IDLE,
        OPENLOOP
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
            case OPENLOOP:
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
            case OPENLOOP:
              return SystemState.OPENLOOP;
            default:
              return SystemState.IDLE;
        }
    }

    public void setWantedState(WantedState wantedState) {
      if (this.wantedState == wantedState) return;
      this.wantedState = wantedState;
    }

    public void setVoltage(double volts) {
      io.setVoltage(volts);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        systemState = handleStateTransitions();
        applyStates();
        Logger.processInputs("Elevator", inputs);
    }

    public Command gotoLevel(WantedState level) {
      return Commands.sequence(
        Commands.run(() -> {
          if(level == WantedState.IDLE) return;
          setWantedState(level);
        }, this),
      Commands.waitUntil(
        (level != WantedState.LEVEL_2)
          ? L2
          : (level != WantedState.DRIVER_STATION)
            ? DS
            : ()  -> Boolean.TRUE
      ));
    }
}
