package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.RedRockTalon;
import frc.robot.Utils3006.SmartDashboardNumber;
import frc.robot.subsystems.Superstructure.Position;

/* TODO
 * Find pos values
 * Tune Slot0
 * Tune MM
 * Tune tolerance
 * Tune scoreBarge
 */

public class Arm extends SubsystemBase {
    private final RedRockTalon armMotor = new RedRockTalon(41, "arm-motor", "*");
    private final CANcoder m_encoder = new CANcoder(42);

    private SmartDashboardNumber armTolerance = new SmartDashboardNumber("arm/arm-tolerance", 0);
    private Position targetPosition = Position.STOW;

    private static Arm instance = null;

    private static Map<Position, SmartDashboardNumber > POSITION_CONVERSIONS = Map.of(
        Position.L4, new SmartDashboardNumber("arm/arm-l4", 0),
        Position.L3, new SmartDashboardNumber("arm/arm-l3", 0),
        Position.L2, new SmartDashboardNumber("arm/arm-l2", 0),
        Position.L1, new SmartDashboardNumber("arm/arm-l1", 0),
        Position.SOURCE, new SmartDashboardNumber("arm/arm-source", 0),
        Position.CORAL_GROUND, new SmartDashboardNumber("arm/arm-coral-ground", 0),
        Position.ALGAE_GROUND, new SmartDashboardNumber("arm/arm-algae-ground", 0),
        Position.PROCESSOR, new SmartDashboardNumber("arm/arm-processor", 0),
        Position.STOW, new SmartDashboardNumber("arm/arm-stow", 0),
        Position.BARGE, new SmartDashboardNumber("arm/arm-barge", 0)
    );

    private Arm(){
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
            .withGravityType(GravityTypeValue.Arm_Cosine)
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(0)
            .withMotionMagicCruiseVelocity(0)
        ).withFeedbackConfigs(
            new FeedbackConfigs()
            .withRemoteCANcoder(m_encoder)
        );
    }

    /**
     * Checks if the arm is at target position
     * @return true if the arm is on target
     */
    public boolean atTarget(){
        return Math.abs(Arm.POSITION_CONVERSIONS.get(targetPosition).getNumber()
        - this.armMotor.motor.getPosition().getValueAsDouble()) < armTolerance.getNumber();
    }
    
    /**
     * Moves the arm to a Position
     * @param pos the Position to move to
     * @return a Command to do so
     */
    public Command goToPosition(Position pos){ // TODO: Ensure no illegal movements
        this.targetPosition = pos;
        return Commands.runOnce(
            () -> {this.armMotor.setMotionMagicPosition(Arm.POSITION_CONVERSIONS.get(pos).getNumber());}
        );
    }

    /**
     * Checks if the arm is in danger of hitting the floor
     * @return true if the arm is too low
     */
    public boolean belowThreshold() {
        return this.armMotor.motor.getPosition().getValueAsDouble() < this.armTolerance.getNumber();
    }

    /**
     * Swings arm to score in the Barge
     * @return a Command to do so
     */
    public Command scoreBarge(){
        return goToPosition(Position.L3); // A bit jank but it should work
    }

    /**
     * Get singleton instance
     * @return the Arm
     */
    public static Arm getInstance()
    {
        if(instance == null)
            instance = new Arm();
        return instance;
    }
}
