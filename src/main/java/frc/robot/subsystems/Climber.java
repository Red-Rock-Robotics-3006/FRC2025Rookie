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

    private SmartDashboardNumber spoolInSpeed = new SmartDashboardNumber("climber/spool in speed", 0);
    private SmartDashboardNumber spoolOutSpeed = new SmartDashboardNumber("climber/spool out speed", 0);
    private double ClimberLimit = 4;// TODO FILLER
    private boolean ClimberForward = false;
    private TalonFX m_ClimberMotor = new TalonFX(0, "*");//TODO FILLER
    private CurrentLimitsConfigs mainClimberCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true);


    private Climber() {

        super("Climber");
        this.m_ClimberMotor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)//TODO FILLER
                .withPeakForwardDutyCycle(1d)//TODO FILLER
                .withPeakReverseDutyCycle(-1d)//TODO FILLER
                .withNeutralMode(NeutralModeValue.Brake)
        );
        this.m_ClimberMotor.getConfigurator().apply(mainClimberCurrentLimitsConfigs);
    }

    public void spoolInClimber() {
        this.m_ClimberMotor.setControl(new DutyCycleOut(spoolInSpeed.getNumber()));
        ClimberForward = true;
    }

    public void spoolOutClimber() {
        this.m_ClimberMotor.setControl(new DutyCycleOut(spoolOutSpeed.getNumber()));
        ClimberForward = false;
    }

    public void stopClimber() {
        this.m_ClimberMotor.setControl(new DutyCycleOut(0));
        ClimberForward = false;
    }

    public boolean checkSpoolInLimit(){
        return (m_ClimberMotor.getPosition().getValueAsDouble() >= ClimberLimit);
    }
    public boolean checkSpoolOutLimit(){
        return (m_ClimberMotor.getPosition().getValueAsDouble() <= 0);
    }

    @Override
    public void periodic() {
    }


    public Command SpoolInCommand()
    {
        return new FunctionalCommand(
            () -> {this.spoolInClimber();},
            null,
            (interrupted) -> {this.stopClimber();},
            // TODO
            // Move this to RobotContainer and add an interrupt condition
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
