package frc.robot.dashboard;

public enum DashboardSubsystem {
    SHOOTER,
    DRIVE,
    INTAKE;

    public String prefix() {
        return switch (this) {
            case SHOOTER -> "2 Shooter";
            case DRIVE -> "3 Drive";
            case INTAKE -> "4 Intake";
        };
    }
}
