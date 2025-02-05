package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.SmartDashboardNumber;

public class Intake extends SubsystemBase {
// make the stuff
    private static Intake instance = null;
    private final TalonFX m_intake1 = new TalonFX(0, "*"); // TODO FILLER
    private final TalonFX m_intake2 = new TalonFX(0,"*");//TODO FILLER
    private final TalonFX m_intakePivot = new TalonFX(0, "*"); //TODO FILLER

    private Slot0Configs pivotSlot0Configs = new Slot0Configs();
    private Slot0Configs intakeSlot0Configs = new Slot0Configs();

    private MotionMagicConfigs pivotMotionConfigs = new MotionMagicConfigs();
    private MotionMagicConfigs intakeMotionConfigs = new MotionMagicConfigs();
//Current limit configs
    private CurrentLimitsConfigs intakeCurrentLimitConfigs = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(80)//TODO FILLER
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(120)// TODO FILLER
        .withStatorCurrentLimitEnable(true);

    private CurrentLimitsConfigs pivotCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)// TODO FILLER
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)//TODO FILLER
            .withStatorCurrentLimitEnable(true);

    private SmartDashboardNumber pivotMotionAccel = new SmartDashboardNumber("pivot/pivot-mm-accel", 0);// TODO FILLER
    private SmartDashboardNumber pivotMotionVel = new SmartDashboardNumber("pivot/pivot-mm-velo", 0);// TODO FILLER

    private SmartDashboardNumber intakeMotionAccel = new SmartDashboardNumber("intake/intake-mm-accel", 0);// TODO FILLER
    private SmartDashboardNumber intakeMotionVel = new SmartDashboardNumber("intake/intake-mm-accel", 0);// TODO FILLER

//PID STUFF
    private SmartDashboardNumber intakeKs = new SmartDashboardNumber("intake/ks", 0);
    private SmartDashboardNumber intakeKa = new SmartDashboardNumber("intake/ka", 0);
    private SmartDashboardNumber intakeKv = new SmartDashboardNumber("intake/kv", 0);//TODO FILLER
    private SmartDashboardNumber intakeKp = new SmartDashboardNumber("intake/kp", 0);
    private SmartDashboardNumber intakeKi = new SmartDashboardNumber("intake/ki", 0);
    private SmartDashboardNumber intakeKd = new SmartDashboardNumber("intake/kd", 0);

    private SmartDashboardNumber pivotKs = new SmartDashboardNumber("pivot/ks", 0);
    private SmartDashboardNumber pivotKa = new SmartDashboardNumber("pivot/ka", 0);//TODO FILLER
    private SmartDashboardNumber pivotKv = new SmartDashboardNumber("pivot/kv", 0);//TODO FILLER
    private SmartDashboardNumber pivotKp = new SmartDashboardNumber("pivot/kp", 1);//TODO FILLER
    private SmartDashboardNumber pivotKi = new SmartDashboardNumber("pivot/ki", 0);
    private SmartDashboardNumber pivotKd = new SmartDashboardNumber("pivot/kd", 0);//TODO FILLER
//A bunch of parameters ig
    private SmartDashboardNumber intakeSpeed = new SmartDashboardNumber("intake/intake-speed", 0); //TODO FILLER

    private SmartDashboardNumber pivotStowPosition = new SmartDashboardNumber("pivot/pivot-stow-position", 1);//TODO FILLER
    private SmartDashboardNumber pivotDeployPosition = new SmartDashboardNumber("pivot/pivot-deploy-position", 30);//TODO FILLER

    private SmartDashboardNumber pivotTolerance = new SmartDashboardNumber("pivot/pivot-tolerance", 0.1);//TODO FILLER
    

    private Intake() {
        super("Intake");
    //Configure motors
        this.m_intake1.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)//TODO FILLER
                .withPeakForwardDutyCycle(1d)
                .withPeakReverseDutyCycle(-1d)
                .withNeutralMode(NeutralModeValue.Brake)
        );

        this.m_intake2.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)//TODO FILLER
                        .withPeakForwardDutyCycle(1d)
                        .withPeakReverseDutyCycle(-1d)
                        .withNeutralMode(NeutralModeValue.Brake));


        this.m_intakePivot.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)//TODO FILLER
                        .withPeakForwardDutyCycle(1d)
                        .withPeakReverseDutyCycle(-1d)
                        .withNeutralMode(NeutralModeValue.Brake));
// Configure motor PID
        this.pivotSlot0Configs = new Slot0Configs()
                .withKS(pivotKs.getNumber())
                .withKA(pivotKa.getNumber())
                .withKV(pivotKv.getNumber())
                .withKP(pivotKp.getNumber())
                .withKI(pivotKi.getNumber())
                .withKD(pivotKd.getNumber());

        this.intakeSlot0Configs = new Slot0Configs()
                .withKS(intakeKs.getNumber())
                .withKA(intakeKa.getNumber())
                .withKV(intakeKv.getNumber())
                .withKP(intakeKp.getNumber())
                .withKI(intakeKi.getNumber())
                .withKD(intakeKd.getNumber());
//motiomagic config
        pivotMotionConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(pivotMotionAccel.getNumber())
            .withMotionMagicCruiseVelocity(pivotMotionVel.getNumber());
        
        intakeMotionConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(intakeMotionAccel.getNumber());
        
        this.m_intake1.getConfigurator().apply(intakeSlot0Configs);
        this.m_intake1.getConfigurator().apply(intakeMotionConfigs);
        this.m_intake2.getConfigurator().apply(intakeSlot0Configs);
        this.m_intake2.getConfigurator().apply(intakeMotionConfigs);
        this.m_intake1.getConfigurator().apply(intakeCurrentLimitConfigs);
        this.m_intake2.getConfigurator().apply(intakeCurrentLimitConfigs);
        this.m_intakePivot.getConfigurator().apply(pivotMotionConfigs);
        this.m_intakePivot.getConfigurator().apply(pivotSlot0Configs);
        this.m_intakePivot.getConfigurator().apply(pivotCurrentLimitsConfigs);


        this.m_intake2.setControl(new Follower(0, true));// TODO FILLER
    }

    public void stowIntake() {
        this.m_intakePivot.setControl(
            new MotionMagicVoltage(this.pivotStowPosition.getNumber()) // fix
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(false)
        );
    }

    public void deployIntake() {
        this.m_intakePivot.setControl(
            new MotionMagicVoltage(this.pivotDeployPosition.getNumber()) // fix
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(false)
        );
    }

    public void enableIntake() {
        this.m_intake1.setControl(new MotionMagicVelocityVoltage(intakeSpeed.getNumber() / 60d) // TODO
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(true));  
    }

    public void reverseIntake() {
        this.m_intake1.setControl(new MotionMagicVelocityVoltage(-intakeSpeed.getNumber() / 60d) // change
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(true));
    }

    public void disableIntake() {
        this.m_intake1.setControl(new MotionMagicVelocityVoltage(0) // change
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(true));
    }

    public void resetPivots() {
        this.m_intakePivot.setControl(new CoastOut());
        this.m_intakePivot.setPosition(0);
    }

     @Override
    public void periodic() {
        if (pivotKs.hasChanged()
        || pivotKv.hasChanged()
        || pivotKp.hasChanged()
        || pivotKi.hasChanged()
        || pivotKd.hasChanged()
        || pivotKa.hasChanged()) {
            pivotSlot0Configs.kS = pivotKs.getNumber();
            pivotSlot0Configs.kV = pivotKv.getNumber();
            pivotSlot0Configs.kP = pivotKp.getNumber();
            pivotSlot0Configs.kI = pivotKi.getNumber();
            pivotSlot0Configs.kD = pivotKd.getNumber();
            pivotSlot0Configs.kA = pivotKa.getNumber();

            if (!Utils.isSimulation()) this.m_intakePivot.getConfigurator().apply(pivotSlot0Configs);
            System.out.println("applied");
        }

        if (pivotMotionAccel.hasChanged() || pivotMotionVel.hasChanged()) {
            pivotMotionConfigs.MotionMagicAcceleration = pivotMotionAccel.getNumber();
            pivotMotionConfigs.MotionMagicCruiseVelocity = pivotMotionVel.getNumber();
            this.m_intakePivot.getConfigurator().apply(pivotMotionConfigs);
        }

        if (intakeKs.hasChanged()
        || intakeKv.hasChanged()
        || intakeKp.hasChanged()
        || intakeKi.hasChanged()
        || intakeKd.hasChanged()
        || intakeKa.hasChanged()) {
            intakeSlot0Configs.kS = intakeKs.getNumber();
            intakeSlot0Configs.kV = intakeKv.getNumber();
            intakeSlot0Configs.kP = intakeKp.getNumber();
            intakeSlot0Configs.kI = intakeKi.getNumber();
            intakeSlot0Configs.kD = intakeKd.getNumber();
            intakeSlot0Configs.kA = intakeKa.getNumber();

            if (!Utils.isSimulation()) {this.m_intake1.getConfigurator().apply(intakeSlot0Configs); this.m_intake2.getConfigurator().apply(intakeSlot0Configs);}
            System.out.println("applied");
        }

        if (intakeMotionAccel.hasChanged()) {
            this.intakeMotionConfigs.MotionMagicAcceleration = intakeMotionAccel.getNumber();
            this.m_intake1.getConfigurator().apply(intakeMotionConfigs);
            this.m_intake2.getConfigurator().apply(intakeMotionConfigs);
        }

        SmartDashboard.putNumber("intake/intake-position", m_intakePivot.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("intake/intake-velo", m_intake1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("intake/intake-velo", m_intake2.getVelocity().getValueAsDouble());

    }
    public Command enableIntakeCommand() {
        return Commands.runOnce(
            () -> this.enableIntake(), this
        );
    }
    public Command reverseIntakeCommand() {
        return Commands.runOnce(
            () -> this.reverseIntake(), this
        );
    }
    public Command disableIntakeCommand() {
        return Commands.runOnce(
            () -> this.disableIntake(), this
        );
    }
    public Command stowIntakeCommand() {
        return Commands.runOnce(
            () -> this.stowIntake(), this
        );
    }
    public Command deployIntakeCommand() {
        return Commands.runOnce(
            () -> this.deployIntake(), this
        );
    }
    public static Intake getInstance() {
        if (Intake.instance == null)
            Intake.instance = new Intake();
        return Intake.instance;
    }
}
