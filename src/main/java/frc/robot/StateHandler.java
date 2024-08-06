package frc.robot;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveStates;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import frc.robot.subsystems.FeederSubsystem.FeederStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeRollerStates;
import frc.robot.subsystems.ShooterSubsystem.ShooterStates;

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
    public ShooterStates desiredShooterState = ShooterStates.IDLE_VELO;
    public IntakeArmStates desiredIntakeArmState = IntakeArmStates.STOWED;
    public IntakeRollerStates desiredIntakeRollerState = IntakeRollerStates.OFF;
    public ArmStates desiredArmState = ArmStates.STOWED;
    public FeederStates desiredFeederState = FeederStates.OFF;

    /* CURRENT STATES: Mechanisms have arrived to the desired state. */
    public ShooterStates currentShooterState = ShooterStates.IDLE_VELO;
    public IntakeArmStates currentIntakeArmState = IntakeArmStates.STOWED;
    public IntakeRollerStates currentIntakeRollerState = IntakeRollerStates.OFF;
    public ArmStates currentArmState = ArmStates.STOWED;
    public FeederStates currentFeederState = FeederStates.OFF;


    /* SWERVE STATES - ONLY CURRENT STATE IS REQUIRED */
    public SwerveStates currentSwerveState = SwerveStates.FIELD_CENTRIC;

    /* BEAM BREAK Values */
    public boolean bb1Covered = false;
    public boolean bb2Covered = false;
    public boolean bb3Covered = false;
    public boolean bb4Covered = false;

    public boolean latchingBB = false;
   

    /* BLOWER PERCENT OUTPUT */
    public double blowerPercent = 0;

    /* HELPER METHODS */
    public boolean hasGamePiece(){
        return bb3Covered;
    }

    // public boolean isCenteredToSpeakerTag(){
    //     return tX that will be stored here < range setup in Limelight constants (maybe this method should be in VisionSubsystem to reduce passing of tX to StateHandler? but it probably should be here to be central? Unless VisionSubsystem was a Singleton?)
    // }

    // public boolean isInSpeakerRange(){
    //     return distance between max and min for hashmap (should this also go in vision subsystem?)
    // }

}
