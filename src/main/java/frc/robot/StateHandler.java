package frc.robot;

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
    }


    public ScoringType scoringType = ScoringType.RANGED;

    /* DESIRED STATES: These tell mechanisms to "go" to the state specified. */
    public ShooterSubsystem.States desiredShooterState = ShooterSubsystem.States.IDLE_VELO;

    /* CURRENT STATES: Mechanisms have arrived to the desired state. */
    public ShooterSubsystem.States currentShooterStates = ShooterSubsystem.States.IDLE_VELO;

    /* SWERVE STUFF */
    public SwerveSubsystem.States currentSwerveState = SwerveSubsystem.States.FIELD_CENTRIC;

    /* BEAM BREAK Values */
    public boolean bb4Covered = false;

    /* BLOWER % */
    public double blowerPercent = 0;



}
