package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.Position;

public class Elevator extends SubsystemBase {
    /**
     * Checks if the elevator is at target position
     * @return true if the elevator is on target
     */
    public boolean atTarget(){return true;}
    /**
     * Moves the elevator to a Position
     * @param pos the position to move to
     * @return a Command to do so
     */
    public Command goToPosition(Position pos){return new Command() {};}
    /**
     * Checks if a Position may drop the arm too low
     * @return true if the Position is below a threshold
     */
    public boolean posBelowThreshold(Position position) {return true;}
    /**
     * Move the endeffector to a normal position and zero it
     * @return a Command to do so
     */
    public Command normalizeCommand() {return new Command() {};}
    /**
     * Get singleton instance
     * @return the Elevator
     */
    public static Elevator getInstance(){return new Elevator();}
}
