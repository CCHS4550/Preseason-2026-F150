package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frx.robot.Constant.Constants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystems extends SubsystemBase{

    public enum WantedIntakeState{
        IDLE,
        INTAKE,
        OUTTAKE
    }
    public enum SystemState{
        IDLE,
        INTAKE,
        OUTTAKE
    }
    private void applyStates()   {
        switch (SystemState){
        case IDLE:     
        IntakeIO.spinnerIntakeVoltage(0.0);
        break;
        case INTAKE:
        IntakeIO.spinnerIntakeVoltage(0.0);
        break;
        case OUTTAKE: 
        IntakeIO.spinnerIntakeVoltage(0.0);
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
}}

public void stWantedIntakeState(WantedIntakeState state){
    if(state==this.wantedState) return;
    this.wantedState=state;
}


@Override
public void periodic()[
intakeIO.updateInputs(inputs);
SystemState=handleStateTransitions();
applyStates();

]

}






    }
}

