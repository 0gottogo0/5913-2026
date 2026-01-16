package frc.robot.constants;

public final class Constants {
    public static final class Controllers {
        public static final int DriverControllerID = 0;
        public static final int ManipulatorControllerID = 1;
        public static final double MoveSlewRateLimiter = 12.00;
        public static final double RotateSlewRateLimiter = 40.00;
        public static final double RotateMagnitude = 0.90;
        public static final double StickDeadzone = 0.02;
    }
    
    public static final class FeederConstants {
        public static enum State {
            Idle,
            Feed,
            Unstick,
            Outtake,
            DumbControl
        }

        public static final double FeedingSpeed = 0.15;
        public static final double UnstickSpeed = 0.30;
        public static final double OuttakeSpeed = 0.40;
    }

    public static final class IntakeConstants {
        public static enum State {
            Idle,
            Intake,
            Agitate,
            Outtake,
            DumbControl
        }

        public static final double IntakingSpeed = 0.30;
        public static final double OuttakingSpeed = 0.45;
    }

    public static final class ShooterConstants {
        public static enum State {
            Idle,
            Shoot,
            Unstick,
            DumbControl
        }

        public static final double IdleRPS = 8.00;
        public static final double UnstickRPS = 15.00;
    }
}
