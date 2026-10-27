package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;

    public ElevatorSubsystem(ElevatorIO io){
        this.io = io;
    }
}
