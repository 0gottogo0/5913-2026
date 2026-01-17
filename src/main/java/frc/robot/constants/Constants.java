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

        public static final double FeedingSpeed = 0;
        public static final double UnstickSpeed = 30;
        public static final double OuttakeSpeed = -40;

        public static final double PIDkV = 0.00;
        public static final double PIDkP = 0.20;
        public static final double PIDkI = 0.00;
        public static final double PIDkD = 0.00;


        public static final int MotorID = 31;
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

    public static final class ShooterConstants {
        public static enum State {
            Idle,
            Spinup,
            Shoot,
            OverTorque,
            Unstick,
            DumbControl
        }

        public static final double RPSThresholdPercent = 0.20;
        public static final double ShootRPSAdjustment = 4.00;
        public static final double OverTorqueRPSAdjustment = 0.00;
        public static final double UnstickRPS = 20.00;

        public static final double PIDkV = 0.00;
        public static final double PIDkP = 0.45;
        public static final double PIDkI = 0.00;
        public static final double PIDkD = 0.00;

        public static final int MotorID = 30;
    }
}
