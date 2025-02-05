package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.SmartDashboardNumber;

public class Elevator extends SubsystemBase {
    private static Elevator instance = null;

    private final TalonFX m_elevatorLeft = new TalonFX(21, "*"); // TODO change motor ID
    private final TalonFX m_elevatorRight = new TalonFX(22, "*"); // TODO change motor ID

    private Slot0Configs elevatorSlot0Configs = new Slot0Configs();
    private MotionMagicConfigs elevatorMotionConfigs = new MotionMagicConfigs();

    private CurrentLimitsConfigs elevatorCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80) // TODO change
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120) // TODO change
            .withStatorCurrentLimitEnable(true);

    private SmartDashboardNumber elevatorMotionAccel = new SmartDashboardNumber("elevator/elevator-mm-accel", 30); // TODO change
    private SmartDashboardNumber elevatorSpeed = new SmartDashboardNumber("elevator/elevator-speed", -3200); // TODO change

    private SmartDashboardNumber elevatorKs = new SmartDashboardNumber("elevator/ks", 0);
    private SmartDashboardNumber elevatorKa = new SmartDashboardNumber("elevator/ka", 0);
    private SmartDashboardNumber elevatorKv = new SmartDashboardNumber("elevator/kv", 0.1); // TODO to be tuned;
    private SmartDashboardNumber elevatorKp = new SmartDashboardNumber("elevator/kp", 0);
    private SmartDashboardNumber elevatorKi = new SmartDashboardNumber("elevator/ki", 0);
    private SmartDashboardNumber elevatorKd = new SmartDashboardNumber("elevator/kd", 0);

    private Elevator() {
        super("Elevator");

        this.m_elevatorLeft.getConfigurator().apply( // TODO change
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withPeakForwardDutyCycle(1d)
                        .withPeakReverseDutyCycle(-1d)
                        .withNeutralMode(NeutralModeValue.Brake));

        this.m_elevatorRight.getConfigurator().apply( // TODO change
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withPeakForwardDutyCycle(1d)
                        .withPeakReverseDutyCycle(-1d)
                        .withNeutralMode(NeutralModeValue.Brake));

        this.elevatorSlot0Configs = new Slot0Configs()
                .withKS(elevatorKs.getNumber())
                .withKA(elevatorKa.getNumber())
                .withKV(elevatorKv.getNumber())
                .withKP(elevatorKp.getNumber())
                .withKI(elevatorKi.getNumber())
                .withKD(elevatorKd.getNumber());

        elevatorMotionConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(elevatorMotionAccel.getNumber());

        this.m_elevatorLeft.getConfigurator().apply(elevatorSlot0Configs);
        this.m_elevatorRight.getConfigurator().apply(elevatorSlot0Configs);
        this.m_elevatorLeft.getConfigurator().apply(elevatorMotionConfigs);
        this.m_elevatorRight.getConfigurator().apply(elevatorMotionConfigs);
        this.m_elevatorLeft.getConfigurator().apply(elevatorCurrentLimitConfigs);
        this.m_elevatorRight.getConfigurator().apply(elevatorCurrentLimitConfigs);

        this.m_elevatorRight.setControl(new Follower(21, true)); // TODO change to id of left motor
    }

    public static Elevator getInstance() {
        if (Elevator.instance == null)
            Elevator.instance = new Elevator();
        return Elevator.instance;
    }
}
