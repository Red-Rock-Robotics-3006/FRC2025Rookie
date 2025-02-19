package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.EndEffector;

public class Superstructure extends SubsystemBase {
    private Elevator elevator = new Elevator();
    private Arm arm = new Arm();
    private EndEffector endEffector = new EndEffector();
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


    public Command goToPosition(Position position)
    {
        return new SequentialCommandGroup(
            this.arm.goToPosition(position),
            new WaitUntilCommand(() -> !(elevator.posBelowThreshold(position)&&arm.posBelowThreshold())),
            this.elevator.goToPosition(position),
            this.endEffector.goToPosition(position)
        );

    }




    
}
