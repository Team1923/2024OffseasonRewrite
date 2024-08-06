package frc.robot;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class StateHandler {
    private static StateHandler stateHandler;

    public static synchronized StateHandler getInstance() {
        if (stateHandler == null) {
            stateHandler = new StateHandler();
        }

        return stateHandler;
    }

    public static enum ScoringType {
        AMP,
        SUBWOOFER,
        REVERSE_SUBWOOFER,
        RANGED,
        TRAP,
        HIGH_PUNT,
        LOW_PUNT,
        UNGUARDABLE,
        CLIMB,
    }

    public ScoringType scoringType = ScoringType.RANGED;

    /* DESIRED STATES: These tell mechanisms to "go" to the state specified. */
    public ShooterSubsystem.States desiredShooterState = ShooterSubsystem.States.IDLE_VELO;
    public IntakeSubsystem.ArmStates desiredIntakeArmState = IntakeSubsystem.ArmStates.STOWED;
    public IntakeSubsystem.RollerStates desiredIntakeRollerState = IntakeSubsystem.RollerStates.OFF;

    /* CURRENT STATES: Mechanisms have arrived to the desired state. */
    public ShooterSubsystem.States currentShooterStates = ShooterSubsystem.States.IDLE_VELO;
    public IntakeSubsystem.ArmStates currentArmStates = IntakeSubsystem.ArmStates.STOWED;
    public IntakeSubsystem.RollerStates currentIntakeRollerState = IntakeSubsystem.RollerStates.OFF;

    /* SWERVE STUFF */
    public SwerveSubsystem.States currentSwerveState = SwerveSubsystem.States.FIELD_CENTRIC;

    /* BEAM BREAK Values */
    public boolean bb4Covered = false;
    public boolean bb1Covered = false;

    /* BLOWER PERCENT */
    public double blowerPercent = 0;

}
