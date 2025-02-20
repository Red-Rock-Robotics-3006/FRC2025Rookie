package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.Position;

public class Elevator extends SubsystemBase {

    public boolean atTarget(){return true;}
    public Command goToPosition(Position pos){return new Command() {};}
    public boolean posBelowThreshold(Position position) {return true;}
}
