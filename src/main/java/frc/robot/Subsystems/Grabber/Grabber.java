package frc.robot.Subsystems.Grabber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.GrabberConstants;

public class Grabber extends SubsystemBase{
    private GrabberIO io;
    private GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

    public enum WantedState{
        IDLE,
        INTAKE,
        OUTTAKE
    }
    public enum SystemState{
        IDLE,
        INTAKE,
        OUTTAKE
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public Grabber(GrabberIO io) {
        this.io = io;
    }
    
    private void applyStates()   {
        switch (systemState){
        case INTAKE:
            io.setVoltage(GrabberConstants.intakeVoltage);
            break;
        case OUTTAKE: 
            io.setVoltage(-GrabberConstants.intakeVoltage);
            break;
        case IDLE:     
            io.setVoltage(0.0);
            break;
        default:
            io.setVoltage(0.0);
            break;
        }
    }


    private SystemState handleStateTransitions(){
        switch (wantedState){
        case IDLE:
            return SystemState.IDLE; 
        case INTAKE:
            return SystemState.INTAKE;
        case OUTTAKE:
            return SystemState.OUTTAKE;
        default:
            return SystemState.IDLE; 
        }
    }

    public void setWantedState(WantedState state){
        if(state == this.wantedState) return;
        this.wantedState = state;
    }


    @Override
    public void periodic(){
        io.updateInputs(inputs);
        systemState = handleStateTransitions();
        applyStates();
    }
}






   

