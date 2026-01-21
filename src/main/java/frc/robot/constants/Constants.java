package frc.robot.constants;

public final class Constants {
    public static final class AutoAim {
        public static final double BlueGoalPos[] = {0.00, 0.00};
        public static final double RedGoalPos[] = {0.00, 0.00};

        public static final double LeftTurretPos[] = {0.00, 0.00};
        public static final double RightTurretPos[] = {0.00, 0.00};
    }

    public static final class Controllers {
        public static final int DriverControllerID = 0;
        public static final int ManipulatorControllerID = 1;
        public static final double MoveSlewRateLimiter = 12.00;
        public static final double RotateSlewRateLimiter = 40.00;
        public static final double RotateMagnitude = 0.90;
        public static final double StickDeadzone = 0.2;
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
        public static final double OuttakingSpeed = -0.45;
    }
    
    public static final class FeederConstants {
        public static enum State {
            Idle,
            Feed,
            Unstick,
            Outtake,
            DumbControl
        }

        // How fast should the feeder be going compared to the shooter
        public static final double FeedingRatio = 0.85;
        public static final double UnstickSpeed = 30;
        public static final double OuttakeSpeed = -40;

        public static final double PIDkV = 0.00;
        public static final double PIDkP = 0.20;
        public static final double PIDkI = 0.00;
        public static final double PIDkD = 0.00;


        public static final int MotorID = 31;
    }

    public static final class ShooterConstants {
        public static enum State {
            Idle,
            Spinup,
            Shoot,
            Unstick,
            DumbControl
        }

        // Used in determing if shooter is up to speed
        public static final double RPSThreshold = 0.20;
        // How much faster should the shooter go to account for ball
        // compression slowing down shooter
        public static final double ShootRPSAdjustment = 0.75;
        public static final double UnstickRPS = 20.00;

        public static final double PIDkV = 0.00;
        public static final double PIDkP = 0.45;
        public static final double PIDkI = 0.00;
        public static final double PIDkD = 0.00;

        public static final int MotorID = 30;
    }
}
