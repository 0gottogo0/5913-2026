package frc.robot.constants;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class Constants {
    public static final class AutoAimConstants {
        public static enum State {
            Goal,
            SnakeDrive,
            DumbControl
        }

        // These are purely guesses based on field cad
        // right now and may need to be re-mesured or
        // adjusted in the future

        // Also note these are using FRC WPIBlue cordinates,
        // more, and better information can be found in the
        // Limelight docs under "3D Coordinate Systems In
        // Detail"
        public static final Pose2d BlueGoal = new Pose2d(4.50,  4.07, new Rotation2d(0));
        public static final Pose2d RedGoal = new Pose2d(11.90, 4.07, new Rotation2d(0));
        public static final Pose2d NeutralZone = new Pose2d(8.27, 4.07, new Rotation2d(0));
        public static final Pose2d BlueZone = new Pose2d(3.20, 4.07, new Rotation2d(0));
        public static final Pose2d RedZone = new Pose2d(13.40, 4.07, new Rotation2d(0));

        // Set to x:0, y:0, r:0 for no turret because
        // the "turret" is just our swerve
        public static final Transform2d TurretRotatePoint = new Transform2d(0, 0, new Rotation2d(0));
        
        // Meters, RPS
        public static final InterpolatingDoubleTreeMap TopShooterSpeedByDistance = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(0.00, 55.00),
            Map.entry(1.30, 55.00),
            Map.entry(1.70, 57.00),
            Map.entry(2.10, 65.00),
            Map.entry(3.40, 87.00) // Tower
            //Map.entry(5.00, 40.00)
            );
    
        // Meters, RPS
        public static final InterpolatingDoubleTreeMap HoodShooterSpeedByDistance = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(0.00, 0.00),
            Map.entry(1.30, 0.00),
            Map.entry(1.70, 0.00),
            Map.entry(2.10, 0.00),
            Map.entry(3.40, 0.00) // Tower
            //Map.entry(5.00, 70.00)
        );
            
        // Meters, Seconds
        /*public static final InterpolatingDoubleTreeMap TimeOfFlightByDistance = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(0.00, 0.50),
            Map.entry(1.64, 0.50),
            Map.entry(2.30, 0.85),
            Map.entry(3.00, 0.98),
            Map.entry(3.60, 1.20),
            Map.entry(4.30, 1.30),
            Map.entry(4.50, 1.70)
        );*/

        public static final double TrackingHubPIDkP = 0.02;
        public static final double TrackingHubPIDkI = 0.00;
        public static final double TrackingHubPIDkD = 0.00;

        public static final String LimelightCenter = "limelight-center";
    }

    public static final class ControllerConstants {
        // Units allowed to change per seccond
        public static final double XSlewRateLimiter = 8.00;
        public static final double YSlewRateLimiter = 8.00;
        public static final double RotateSlewRateLimiter = 24.00;

        public static final double RotateMagnitude = 0.90;
        public static final double StickDeadzone = 0.10;

        public static final double IntakeAgitateTime = 1.00;

        public static final int XboxMenuButtonID = 7;
        public static final int XboxShareButtonID = 8;

        public static final int DriverControllerID = 0;
        public static final int ManipulatorControllerID = 1;
        public static final int TestingControllerID = 2;
    }
    
    public static final class IntakeConstants {
        public static enum State {
            IdleIn,
            IdleOut,
            IntakeIn,
            IntakeOut,
            Outtake,
            Agitate,
            DumbControl
        }
        
        // Tune this
        public static final double IntakePIDkP = 0.0003;
        public static final double IntakePIDkI = 0.00;
        public static final double IntakePIDkD = 0.0000;
        public static final int IntakeCurrentLimit = 80;

        public static final double PivotPIDkP = 3.00;
        public static final double PivotPIDkI = 0.00;
        public static final double PivotPIDkD = 0.00;
        public static final double PivotCurrentLimit = 60;

        public static final double IntakingSpeed = 5000.00; 

        public static final double PivotRetractSpeed = 1.00;
        public static final double PivotExtensionSpeed = 0.75;
        public static final double PivotAgitateSpeed = 0.30;

        public static final double PivotInPos = 0.60;
        public static final double PivotInTriggerPos = 0.48;
        public static final double PivotRunIntakeTriggerPos = 0.35;
        public static final double PivotOutPos = 0.24;

        public static final int IntakeLeftID = 34;
        public static final int IntakeRightID = 50;
        public static final int PivotID = 37;
    }

    public static final class ShooterConstants {
        public static enum State {
            Idle,
            Spinup,
            Shoot,
            Unstick,
            DumbControl
        }
        
        public static final double BeltsSpeed = 35.00;
        public static final double FeedSpeed = 35.00;
        public static final double BottomShooterSpeed = 38.50;
        
        // Used in determing if shooter is up to speed
        public static final double RPSThreshold = 1.25;

        public static final double BeltsPIDkV = 0.105;
        public static final double BeltsPIDkP = 0.20;
        public static final double BeltsPIDkI = 0.00;
        public static final double BeltsPIDkD = 0.00;
        public static final double BeltsCurrentLimit = 40;
        
        public static final double FeederPIDkV = 0.135;
        public static final double FeederPIDkP = 0.30;
        public static final double FeederPIDkI = 0.00;
        public static final double FeederPIDkD = 0.00;
        public static final double FeederCurrentLimit = 80;

        public static final double BottomShooterPIDkV = 0.135;
        public static final double BottomShooterPIDkP = 0.30;
        public static final double BottomShooterPIDkI = 0.00;
        public static final double BottomShooterPIDkD = 0.00;
        public static final double BottomShooterCurrentLimit = 40;

        public static final double LeftShooterPIDkV = 0.122;
        public static final double LeftShooterPIDkP = 0.30;
        public static final double LeftShooterPIDkI = 0.00;
        public static final double LeftShooterPIDkD = 0.00;
        public static final double LeftShooterCurrentLimit = 80;

        public static final double RightShooterPIDkV = 0.122;
        public static final double RightShooterPIDkP = 0.30;
        public static final double RightShooterPIDkI = 0.00;
        public static final double RightShooterPIDkD = 0.00;
        public static final double RightShooterCurrentLimit = 80;

        public static final double HoodShooterPIDkV = 0.122;
        public static final double HoodShooterPIDkP = 0.30;
        public static final double HoodShooterPIDkI = 0.00;
        public static final double HoodShooterPIDkD = 0.00;
        public static final double HoodShooterCurrentLimit = 70;

        public static final int BottomRollersID = 45;
        public static final int FeederMotorID = 43;
        public static final int BottomMotorID = 39;
        public static final int LeftMotorID = 40;
        public static final int RightMotorID = 48;
        public static final int HoodMotorID = 47;
    }
}
