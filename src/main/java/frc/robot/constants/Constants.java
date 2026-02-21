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
            NeutralZone,
            AllianceZone,
            ClimbLeft,
            ClimbRight,
            DumbControl
        }

        // These are purely guesses based on field cad
        // right now and may need to be re-mesured or
        // adjusted in the future

        // Also note these are using FRC WPIBlue, more
        // and better information can be found in the
        // Limelight docs under "3D Coordinate Systems
        // In Detail"
        public static final Pose2d BlueGoal = new Pose2d(4.60,  4.07, new Rotation2d(0));
        public static final Pose2d RedGoal = new Pose2d(11.60, 4.07, new Rotation2d(0));
        public static final Pose2d NeutralZone = new Pose2d(8.27, 4.07, new Rotation2d(0));
        public static final Pose2d BlueZone = new Pose2d(3.20, 4.07, new Rotation2d(0));
        public static final Pose2d RedZone = new Pose2d(13.40, 4.07, new Rotation2d(0));

        public static final Pose2d BlueClimbLeft = new Pose2d(0, 0, new Rotation2d(0));
        public static final Pose2d BlueClimbRight = new Pose2d(0, 0, new Rotation2d(0));
        public static final Pose2d RedClimbLeft = new Pose2d(0, 0, new Rotation2d(0));
        public static final Pose2d RedClimbRight = new Pose2d(0, 0, new Rotation2d(0));

        // Set to x:0, y:0, r:0 for no turret because
        // the "turret" is just our swerve
        public static final Transform2d TurretRotatePoint = new Transform2d(0, 0, new Rotation2d(0));
        
        // Meters, RPS
        public static final InterpolatingDoubleTreeMap TopShooterSpeedByDistance = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(0.00, 0.00),
            Map.entry(0.50, 0.00),
            Map.entry(1.00, 0.00),
            Map.entry(1.50, 0.00),
            Map.entry(2.00, 0.00),
            Map.entry(2.50, 0.00),
            Map.entry(3.00, 0.00),
            Map.entry(3.50, 0.00),
            Map.entry(4.00, 0.00),
            Map.entry(4.50, 0.00),
            Map.entry(5.00, 0.00)
            );
    
        // Meters, RPS
        public static final InterpolatingDoubleTreeMap HoodShooterSpeedByDistance = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(0.00, 0.00),
            Map.entry(0.50, 0.00),
            Map.entry(1.00, 0.00),
            Map.entry(1.50, 0.00),
            Map.entry(2.00, 0.00),
            Map.entry(2.50, 0.00),
            Map.entry(3.00, 0.00),
            Map.entry(3.50, 0.00),
            Map.entry(4.00, 0.00),
            Map.entry(4.50, 0.00),
            Map.entry(5.00, 0.00)
        );
            
        // Meters, Seconds
        public static final InterpolatingDoubleTreeMap TimeOfFlightByDistance = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(0.00, 0.00),
            Map.entry(0.50, 0.00),
            Map.entry(1.00, 0.00),
            Map.entry(1.50, 0.00),
            Map.entry(2.00, 0.00),
            Map.entry(2.50, 0.00),
            Map.entry(3.00, 0.00),
            Map.entry(3.50, 0.00),
            Map.entry(4.00, 0.00),
            Map.entry(4.50, 0.00),
            Map.entry(5.00, 0.00)
        );

        public static final double TrackingHubPIDkP = 0.00;
        public static final double TrackingHubPIDkI = 0.00;
        public static final double TrackingHubPIDkD = 0.00;

        public static final double TrackingClimbMovePIDkP = 0.00;
        public static final double TrackingClimbMovePIDkI = 0.00;
        public static final double TrackingClimbMovePIDkD = 0.00;

        public static final double TrackingClimbRotPIDkP = 0.00;
        public static final double TrackingClimbRotPIDkI = 0.00;
        public static final double TrackingClimbRotPIDkD = 0.00;

        public static final String LimelightCenter = "limelight-center";
        public static final String LimelightRight = "llright";
        public static final String LimelightClimb = "llclimb";
    }

    public static final class ClimberConstants {
        public static enum State {
            Idle,
            ClimbUp,
            ClimbDown,
            DumbControl
        }

        // How many rotations of the pivot motor does it take
        // to move the hooks a degree 
        public static final double PivotMotorRotationsToOneDegree = 0.00;

        public static final double ClimbUpSetpoint =  0.00;
        public static final double ClimbDownSetpoint = 0.00;

        public static final double PIDkG = 0.00;
        public static final double PIDkP = 0.20;
        public static final double PIDkI = 0.00;
        public static final double PIDkD = 0.00;

        public static final int MotorID = 48;
    }

    public static final class ControllerConstants {
        public static enum DrivetrainState {
            DisabledDrivetrain,
            BabyMode,
            SlowTC,
            EventTC,
            GoCrazyGoStupid, // Thank you team 4539 for this great
                             // name idea at the 2025 NMRC Chamionship
            HubTracking,
            ClimbTracking      
        }

        // Units allowed to change per seccond
        public static final double XSlewRateLimiter = 12.00;
        public static final double YSlewRateLimiter = 12.00;
        public static final double RotateSlewRateLimiter = 40.00;

        public static final double RotateMagnitude = 0.90;
        public static final double StickDeadzone = 0.20;

        public static final int XboxMenuButtonID = 7;
        public static final int XboxShareButtonID = 8;

        public static final int DriverControllerID = 0;
        public static final int ManipulatorControllerID = 1;
    }
    
    public static final class IntakeConstants {
        public static enum State {
            Idle,
            IdleOut,
            Intake,
            Unstick,
            Outtake,
            DumbControl
        }
        
        public static final double IntakePIDkV = 0.00;
        public static final double IntakePIDkP = 0.20;
        public static final double IntakePIDkI = 0.00;
        public static final double IntakePIDkD = 0.00;

        public static final double PivotPIDkG = 0.00;
        public static final double PivotPIDkP = 0.20;
        public static final double PivotPIDkI = 0.00;
        public static final double PivotPIDkD = 0.00;

        public static final int IntakeCurrentLimit = 120;
        public static final double IntakingSpeed = 0.42;

        public static final int HopperIn = 1;
        public static final int HopperOut = 0;

        public static final double PivotInPos = 0.42;
        public static final double PivotOutPos = 0.08;
        public static final double SafeToRetractHopperPos = 0.28;

        public static final int IntakeID = 34;
        public static final int PivotID = 37;
    }
    
    public static final class PneumaticConstants {
        public static final int PneumaticsHubID = 17;
    }

    public static final class ShooterConstants {
        public static enum State {
            Idle,
            Spinup,
            Shoot,
            Unstick,
            DumbControl
        }

        public static final double BottomRatio = 1.10;
        public static final double BeltsSpeed = 80.00;
        public static final double FeedSpeed = 40.00;
        public static final double UnstickRPS = 20.00;
        
        // Used in determing if shooter is up to speed
        public static final double RPSThreshold = 1.00;

        public static final double BeltsPIDkV = 0.11;
        public static final double BeltsPIDkP = 0.15;
        public static final double BeltsPIDkI = 0.00;
        public static final double BeltsPIDkD = 0.00;
        
        public static final double FeederPIDkV = 0.11;
        public static final double FeederPIDkP = 0.15;
        public static final double FeederPIDkI = 0.00;
        public static final double FeederPIDkD = 0.00;

        public static final double BottomShooterPIDkV = 0.11;
        public static final double BottomShooterPIDkP = 0.40;
        public static final double BottomShooterPIDkI = 0.00;
        public static final double BottomShooterPIDkD = 0.00;

        public static final double TopShooterPIDkV = 0.13;
        public static final double TopShooterPIDkP = 0.40;
        public static final double TopShooterPIDkI = 0.00;
        public static final double TopShooterPIDkD = 0.00;

        public static final double HoodShooterPIDkV = 0.11;
        public static final double HoodShooterPIDkP = 0.40;
        public static final double HoodShooterPIDkI = 0.00;
        public static final double HoodShooterPIDkD = 0.00;

        public static final int BeltsID = 45;
        public static final int FeederMotorID = 43;
        public static final int BottomMotorID = 39;
        public static final int TopMotorID = 40;
        public static final int HoodMotorID = 47;
    }
}
