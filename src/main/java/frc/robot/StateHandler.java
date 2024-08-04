package frc.robot;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;

import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class StateHandler {
    private static StateHandler stateHandler;

    public static synchronized StateHandler getInstance() {
        if (stateHandler == null) {
            stateHandler = new StateHandler();
        }

        return stateHandler;
    }

    public static enum ScoringType{
        AMP,
        SUBWOOFER,
        REVERSE_SUBWOOFER,
        RANGED,
        TRAP,
        HIGH_PUNT,
        LOW_PUNT,
        UNGUARDABLE,
        CLIMB,
        FULL_EJECT
    }


    public ScoringType scoringType = ScoringType.RANGED;

    /* DESIRED STATES: These tell mechanisms to "go" to the state specified. */
    public ShooterSubsystem.States desiredShooterState = ShooterSubsystem.States.IDLE;
    public IntakeSubsystem.ArmStates desiredIntakeArmState = IntakeSubsystem.ArmStates.STOWED;
    public IntakeSubsystem.RollerStates desiredIntakeRollerState = IntakeSubsystem.RollerStates.OFF;

    /* CURRENT STATES: Mechanisms have arrived to the desired state. */
    public ShooterSubsystem.States currentShooterStates = ShooterSubsystem.States.IDLE;
    public IntakeSubsystem.ArmStates currentArmStates = IntakeSubsystem.ArmStates.STOWED;
    public IntakeSubsystem.RollerStates currentIntakeRollerState = IntakeSubsystem.RollerStates.OFF;

    /* BEAM BREAK Values */
    public boolean bb4Covered = false;
    public boolean bb1Covered = false;

    /*BLOWER % */
    public double blowerPercent = 0;

}
