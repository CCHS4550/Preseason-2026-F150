package frc.robot.Subsystems.Grabber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class Grabber extends SubsystemBase{
    private GrabberIO io;
    private GrabberIOInputsAutologged inputs = new GrabberIOInputsAutologged();

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
    
    private void applyStates()   {
        switch (systemState){
        case INTAKE:
            io.setSpinnerVoltage(GrabberConstants.intakeVoltage);
            break;
        case OUTTAKE: 
            io.setSpinnerVoltage(-GrabberConstants.intakeVoltage);
            break;
        case IDLE:     
            io.setSpinnerVoltage(0.0);
            break;
        default:
            io.setSpinnerVoltage(0.0);
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






   

