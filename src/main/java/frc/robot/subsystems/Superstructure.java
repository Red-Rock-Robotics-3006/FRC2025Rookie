package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

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
        STOW,
        BARGE
    }


    private Command goToPosition(Position position)
    {
        return new SequentialCommandGroup(
            this.arm.goToPosition(position),
            new WaitUntilCommand(() -> !(elevator.posBelowThreshold(position)&&arm.posBelowThreshold())),
            this.elevator.goToPosition(position),
            this.endEffector.goToPosition(position)
            );
        }
        private Command intakeCoral(){
            return new InstantCommand(
                () -> this.endEffector.intakeCoral(),
                this.endEffector
                
            );    
    }
    private Command intakeAlgae(){
        return new InstantCommand(
            () -> this.endEffector.intakeAlgae(),
            this.endEffector
            );    
    }
    private Command scoreCoral(){
        return new InstantCommand(
            () -> this.endEffector.scoreCoral(),
            this.endEffector
            );    
    }
    private Command scoreAlgae(){
        return new InstantCommand(
            () -> this.endEffector.scoreAlgae(),
            this.endEffector
            );    
    }
    private Command scoreBarge(){
        return new SequentialCommandGroup(
            this.elevator.goToPosition(Position.BARGE),
            this.arm.goToPosition(Position.BARGE),
            new WaitUntilCommand(() -> !(elevator.atTarget()&&arm.atTarget())),
            new WaitCommand(.05),
            this.arm.scoreBarge(),
            new WaitCommand(.5),
            this.endEffector.scoreBarge()
        );
    }
    
}