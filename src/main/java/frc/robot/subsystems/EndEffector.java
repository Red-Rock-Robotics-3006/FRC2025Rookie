package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    private final RedRockTalon endEffectorMotor = new RedRockTalon(51,"endeffector-motor",("*"));//TODO
    private final RedRockTalon wristMotor = new RedRockTalon(52,"endeffector/wrist",("*"));//TODO
    private SmartDashboardNumber coralIntakeSpeed = new SmartDashboardNumber("endeffector/coralintakespeed", 0);
    private SmartDashboardNumber coralOuttakeSpeed = new SmartDashboardNumber("endeffector/coralintakespeed", 0);
    private final CANrange endEffectorCoralRange = new CANrange(53); //TODO
    private SmartDashboardNumber coralThreshold = new SmartDashboardNumber("endeffector/coralthreshold", 0);

    private SmartDashboardNumber currentThreshold = new SmartDashboardNumber("endeffector/currentThreshold", 0);
    private SmartDashboardNumber algaeIntakeSpeed = new SmartDashboardNumber("endeffector/algaeIntakeSpeed", -0.2);
    private SmartDashboardNumber algaeOuttakeSpeed = new SmartDashboardNumber("endeffector/algaeOuttakeSpeed", 0);

    public boolean atTarget(){return true;}
    public Command goToPosition(Position pos){return new Command() {};}
    public Command scoreBarge(){ return new Command() {};};
    
    private EndEffector(){
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
    private boolean atCurrentSpike(){
        return Math.abs(this.endEffectorMotor.motor.getTorqueCurrent().getValueAsDouble()) > this.currentThreshold.getNumber();
    }

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
            () -> atCurrentSpike(),
            this
        );
    }
    public Command outakeCoral(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setSpeed(coralOuttakeSpeed.getNumber())),
            new WaitCommand(.25), //TODO FILLER
            Commands.runOnce(() -> setSpeed(0))
        );
    }
    public Command outakeAlgae(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setSpeed(algaeOuttakeSpeed.getNumber())),
            new WaitCommand(.25), //TODO FILLER
            Commands.runOnce(() -> setSpeed(0))
        );
    }
}
