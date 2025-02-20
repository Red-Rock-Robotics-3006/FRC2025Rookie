package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.RedRockTalon;
import frc.robot.Utils3006.SmartDashboardNumber;
import frc.robot.subsystems.Superstructure.Position;

public class EndEffector extends SubsystemBase {
    private final RedRockTalon endEffectorMotor = new RedRockTalon(0,"endeffector-motor",("*"));//TODO
    private final RedRockTalon wristMotor = new RedRockTalon(0,"endeffector-wrist",("*"));//TODO
    private SmartDashboardNumber coralIntakeSpeed

    public boolean atTarget(){return true;}
    public Command goToPosition(Position pos){return new Command() {};}
    public void intakeCoral(){}
    public void scoreCoral(){}
    public void intakeAlgae(){}
    public void scoreAlgae(){}
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
    public Command intakeCoral(){
        return new FunctionalCommand(
            () -> setSpeed(0)
            , null, null, null, null)
    }
    public Command intakeAlgae(){
        
    }
    public Command outakeCoral(){
        
    }
    public Command outakeAlgae(){
        
    }
}
