package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Utils3006.RedRockTalon;
import frc.robot.Utils3006.SmartDashboardNumber;
import frc.robot.subsystems.Superstructure.Position;

public class EndEffector extends SubsystemBase {
    private final RedRockTalon endEffectorMotor = new RedRockTalon(51,"endeffector-motor",("*"));
    private final RedRockTalon wristMotor = new RedRockTalon(52,"endeffector/wrist",("*"));
    private SmartDashboardNumber coralIntakeSpeed = new SmartDashboardNumber("endeffector/coralintakespeed", 0);
    private SmartDashboardNumber coralOuttakeSpeed = new SmartDashboardNumber("endeffector/coralintakespeed", 0);
    private final CANrange endEffectorCoralRange = new CANrange(53);
    private SmartDashboardNumber coralThreshold = new SmartDashboardNumber("endeffector/coralthreshold", 0);
    private SmartDashboardNumber normalizeSpeed = new SmartDashboardNumber("endeffector/normalizespeed", -0.1);
    private SmartDashboardNumber algaeIntakeSpeed = new SmartDashboardNumber("endeffector/algaeIntakeSpeed", -0.2);
    private SmartDashboardNumber algaeOuttakeSpeed = new SmartDashboardNumber("endeffector/algaeOuttakeSpeed", 0);
    private SmartDashboardNumber wristTolerance = new SmartDashboardNumber("endeffector/wristTolerance", 0.1);
    private Position currentPosition = Position.STOW;

    private static Map<Position, SmartDashboardNumber > POSITION_CONVERSIONS = Map.of(
        Position.L4, new SmartDashboardNumber("endeffector/wrist-l4", 0),
        Position.L3, new SmartDashboardNumber("endeffector/wrist-l3", 0),
        Position.L2, new SmartDashboardNumber("endeffector/wrist-l2", 0),
        Position.L1, new SmartDashboardNumber("endeffector/wrist-l1", 0),
        Position.SOURCE, new SmartDashboardNumber("endeffector/wrist-SOURCE", 0),
        Position.CORAL_GROUND, new SmartDashboardNumber("endeffector/wrist-CORAL_GROUND", 0),
        Position.ALGAE_GROUND, new SmartDashboardNumber("endeffector/wrist-ALGAE_GROUND", 0),
        Position.PROCESSOR, new SmartDashboardNumber("endeffector/wrist-PROCESSOR", 0),
        Position.STOW, new SmartDashboardNumber("endeffector/wrist-STOW", 0),
        Position.BARGE, new SmartDashboardNumber("endeffector/wrist-BARGE", 0)
    );
    public EndEffector(){
        super("End Effector");
        endEffectorMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(0)
            .withKI(0)
            .withKD(0)
        )
        .withSpikeThreshold(55); // TODO
                
        wristMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(0)
            .withKI(0)
            .withKD(0)
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(0)
            .withMotionMagicCruiseVelocity(0)
        )
        .withSpikeThreshold(55); // TODO        
    }
    public void setSpeed(double speed){
        this.endEffectorMotor.motor.set(speed);
    }
    
    public boolean atTarget(){
        return Math.abs(EndEffector.POSITION_CONVERSIONS.get(currentPosition).getNumber() - this.wristMotor.motor.getPosition().getValueAsDouble()) < wristTolerance.getNumber();
    }
    public Command goToPosition(Position pos){
        return Commands.runOnce(() -> this.wristMotor.setMotionMagicPosition(EndEffector.POSITION_CONVERSIONS.get(pos).getNumber()), this);
    }

    public Command scoreBarge(){ return new Command() {};};

    private boolean coralDetected(){
        return endEffectorCoralRange.getDistance().getValueAsDouble() < coralThreshold.getNumber();
    }
    public Command intakeCoral(){
        return new FunctionalCommand(
            () -> setSpeed(coralIntakeSpeed.getNumber()),
            () -> {},
            (interrupted) -> setSpeed(0),
            () -> coralDetected(),
            this
            );
        }
        public Command intakeAlgae(){
            return new FunctionalCommand(
                () -> setSpeed(algaeIntakeSpeed.getNumber()),
                () -> {},
                (interrupted) -> setSpeed(0),
                () -> this.endEffectorMotor.aboveSpikeThreshold(),
                this
                );
            }
            public Command outakeCoral(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setSpeed(coralOuttakeSpeed.getNumber())),
            new WaitCommand(.25),
            Commands.runOnce(() -> setSpeed(0))
            );
        }
        public Command outakeAlgae(){
            return new SequentialCommandGroup(
                Commands.runOnce(() -> setSpeed(algaeOuttakeSpeed.getNumber())),
            new WaitCommand(.25),
            Commands.runOnce(() -> setSpeed(0))
        );
    }
    public Command normalizeCommand(){
        return new FunctionalCommand(
            () -> setSpeed(normalizeSpeed.getNumber()),
            () -> {},
            (interrupted) -> {setSpeed(0);
                this.wristMotor.motor.setPosition(0);
            },
            () -> this.wristMotor.aboveSpikeThreshold(),
            this
        );
    }
}
