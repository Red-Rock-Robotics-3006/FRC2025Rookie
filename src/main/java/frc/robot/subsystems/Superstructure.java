package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/* TODO
 * Tune scoreBarge delays
 */

public class Superstructure {
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
        STOW,
        BARGE
    }

    public Superstructure()
    {
        this.initialize();
    }

    public Command initialize()
    {
        return new ParallelCommandGroup(
            this.endEffector.normalizeCommand(),
            this.arm.goToPosition(Position.STOW),
            this.elevator.normalizeCommand()
        );
    }

    public Command goToPosition(Position position)
    {
        return new SequentialCommandGroup(
            this.arm.goToPosition(position),
            new WaitUntilCommand(() -> !(elevator.posBelowThreshold(position) && arm.belowThreshold())),
            this.elevator.goToPosition(position),
            this.endEffector.goToPosition(position)
        );
    }

    public Command intakeCoral(){
        return new InstantCommand(
            () -> this.endEffector.intakeCoral(),
            this.endEffector
        );    
    }

    public Command intakeAlgae(){
        return new InstantCommand(
            () -> this.endEffector.intakeAlgae(),
            this.endEffector
        );
    }

    public Command scoreCoral(){
        return this.endEffector.outtakeCoral();
    }

    public Command scoreAlgae(){
        return this.endEffector.outtakeAlgae();
    }

    public Command scoreBarge(){
        return new SequentialCommandGroup(
            this.elevator.goToPosition(Position.BARGE),
            this.arm.goToPosition(Position.BARGE),
            new WaitUntilCommand(() -> this.atTargets()),
            new WaitCommand(.1),
            this.arm.scoreBarge(),
            new WaitCommand(.3),
            this.endEffector.scoreBarge()
        );
    }

    public boolean atTargets()
    {
        return this.elevator.atTarget() && this.arm.atTarget() && this.endEffector.atTarget();
    }

    public Command autoScoreCoral(Position pos)
    {
        return new SequentialCommandGroup(
            this.goToPosition(pos),
            new WaitUntilCommand(() -> this.atTargets()),
            this.endEffector.outtakeCoral(), //TODO see if this SCG waits for the EE's SCG
            new WaitUntilCommand(() -> this.endEffector.idle()), // If so this can be removed
            this.goToPosition(Position.STOW)
        );
    }
    
}