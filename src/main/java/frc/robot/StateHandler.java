package frc.robot;

import frc.robot.subsystems.FeederSubsystem;
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
    public FeederSubsystem.States desiredFeederState = FeederSubsystem.States.OFF;

    /* CURRENT STATES: Mechanisms have arrived to the desired state. */
    public ShooterSubsystem.States currentShooterState = ShooterSubsystem.States.IDLE_VELO;
    public FeederSubsystem.States currentFeederState = FeederSubsystem.States.OFF;

    /* SWERVE STUFF */
    public SwerveSubsystem.States currentSwerveState = SwerveSubsystem.States.FIELD_CENTRIC;

    /* BEAM BREAK Values */
    public boolean bb2Covered = false;
    public boolean bb3Covered = false;
    public boolean bb4Covered = false;

    /* BLOWER PERCENT */
    public double blowerPercent = 0;


    /* HELPER METHODS */
    public boolean hasGamePiece(){
        return bb3Covered;
    }

    // public boolean isCenteredToSpeakerTag(){
    //     return tX that will be stored here < range setup in Limelight constants (maybe this method should be in VisionSubsystem to reduce passing of tX to StateHandler?)
    // }

    // public boolean isInSpeakerRange(){
    //     return distance between max and min for hashmap (should this also go in vision subsystem?)
    // }

}
