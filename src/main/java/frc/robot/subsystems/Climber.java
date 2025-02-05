package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.SmartDashboardNumber;

public class Climber extends SubsystemBase{

    private TalonFX m_ClimberMotor = new TalonFX(0, "*"); //TODO FILLER

    private SmartDashboardNumber spoolInSpeed = new SmartDashboardNumber("climber/spoolInSpeed", 0.1);
    private SmartDashboardNumber spoolOutSpeed = new SmartDashboardNumber("climber/spoolOutSpeed", -0.1);

    private double climberLimit = 4; // TODO FILLER

    private CurrentLimitsConfigs mainClimberCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true);


    private Climber() {
        super("Climber");


        this.m_ClimberMotor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive) //TODO FILLER
                .withPeakForwardDutyCycle(1d) //TODO FILLER
                .withPeakReverseDutyCycle(-1d) //TODO FILLER
                .withNeutralMode(NeutralModeValue.Brake)
        );
        this.m_ClimberMotor.getConfigurator().apply(mainClimberCurrentLimitsConfigs);
    }

    public void spoolInClimber() {
        if(!checkSpoolInLimit())
            this.m_ClimberMotor.setControl(new DutyCycleOut(spoolInSpeed.getNumber()));
    }

    public void spoolOutClimber() {
        if(!checkSpoolOutLimit())
            this.m_ClimberMotor.setControl(new DutyCycleOut(spoolOutSpeed.getNumber()));
    }

    public void stopClimber() {
        this.m_ClimberMotor.setControl(new DutyCycleOut(0));
    }

    public boolean checkSpoolInLimit(){
        return m_ClimberMotor.getPosition().getValueAsDouble() >= climberLimit;
    }

    public boolean checkSpoolOutLimit(){
        return m_ClimberMotor.getPosition().getValueAsDouble() <= 0;
    }


    public Command SpoolInCommand()
    {
        return new FunctionalCommand(
            () -> {this.spoolInClimber();},
            null,
            (interrupted) -> {this.stopClimber();},
            // TODO
            // Move these to RobotContainer and add an interrupt condition
            // of the button that activated it being released
            () -> {return this.checkSpoolInLimit();},
            this);
    }

    public Command SpoolOutCommand()
    {
        return new FunctionalCommand(
            () -> {this.spoolOutClimber();},
            null,
            (interrupted) -> {this.stopClimber();},
            () -> {return this.checkSpoolOutLimit();},
            this);
    }
}
