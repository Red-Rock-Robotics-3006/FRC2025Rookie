package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    public static enum Position {
        L1,
        L2,
        L3,
        L4,
        SOURCE,
        CORAL_GROUND,
        ALGAE_GROUND,
        PROCESSOR,
        STOW
    }

    public boolean atTarget(){return true;}
    public void goToPosition(Position pos){};
}
