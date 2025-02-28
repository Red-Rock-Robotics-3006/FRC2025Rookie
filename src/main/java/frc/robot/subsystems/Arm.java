package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.RedRockTalon;
import frc.robot.Utils3006.SmartDashboardNumber;
import frc.robot.subsystems.Superstructure.Position;

public class Arm extends SubsystemBase {
    private final RedRockTalon armMotor = new RedRockTalon(41, "arm-Motor", ("*"));
    private SmartDashboardNumber armTolerance = new SmartDashboardNumber("arm-tolerance", 0);
    private static Map<Position, SmartDashboardNumber > POSITION_CONVERSIONS = Map.of(
        Position.L4, new SmartDashboardNumber("arm/-l4", 0),
        Position.L3, new SmartDashboardNumber("arm/-l3", 0),
        Position.L2, new SmartDashboardNumber("arm/-l2", 0),
        Position.L1, new SmartDashboardNumber("arm/-l1", 0),
        Position.SOURCE, new SmartDashboardNumber("arm/-SOURCE", 0),
        Position.CORAL_GROUND, new SmartDashboardNumber("arm/-CORAL_GROUND", 0),
        Position.ALGAE_GROUND, new SmartDashboardNumber("arm/-ALGAE_GROUND", 0),
        Position.PROCESSOR, new SmartDashboardNumber("arm/-PROCESSOR", 0),
        Position.STOW, new SmartDashboardNumber("arm/-STOW", 0),
        Position.BARGE, new SmartDashboardNumber("arm/-BARGE", 0)
        
    );
    private Position currentPosition = Position.STOW;
    public Arm(){
        super("Arm");
        armMotor.withMotorOutputConfigs(
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
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(0)
            .withMotionMagicCruiseVelocity(0)
        );
    }

    public boolean atTarget(){
        return Math.abs(Arm.POSITION_CONVERSIONS.get(currentPosition).getNumber() - this.armMotor.motor.getPosition().getValueAsDouble()) < armTolerance.getNumber();
    }
    
    public Command goToPosition(Position pos){
        return Commands.runOnce(
            () -> {this.armMotor.setMotionMagicPosition(Arm.POSITION_CONVERSIONS.get(pos).getNumber());}
        );
    }
    public boolean posBelowThreshold() {return true;}
    public Command scoreBarge(){ return new Command() {};}; // Don't do this
}
