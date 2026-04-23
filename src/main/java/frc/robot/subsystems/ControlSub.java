// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.Constants.AutoAimConstants;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ShooterConstants.State;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.ControllerConstants.GuitarButtons;
import frc.robot.constants.Constants.ControllerConstants.ManipulatorControllerType;
import frc.robot.constants.TunerConstants;

public class ControlSub extends SubsystemBase {

    // Something with our drivetrain is fundamentally backwards and
    // I dont know what it is
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public double maxAngularRate = RotationsPerSecond.of(ControllerConstants.RotateMagnitude).in(RadiansPerSecond);
    
    // Drive with controller request
    private final SwerveRequest.FieldCentric ControllerDrive = new SwerveRequest.FieldCentric()
        //.withDeadband(maxSpeed * Controllers.StickDeadzone).withRotationalDeadband(maxAngularRate * Controllers.StickDeadzone)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SlewRateLimiter XSlewRateLimiter = new SlewRateLimiter(ControllerConstants.XSlewRateLimiter);
    private final SlewRateLimiter YSlewRateLimiter = new SlewRateLimiter(ControllerConstants.YSlewRateLimiter);
    private final SlewRateLimiter RotateSlewRateLimiter = new SlewRateLimiter(ControllerConstants.RotateSlewRateLimiter);

    public final PIDController HubTrackingPidController = new PIDController(AutoAimConstants.TrackingHubPIDkP, AutoAimConstants.TrackingHubPIDkI, AutoAimConstants.TrackingHubPIDkD);
    
    private final CommandXboxController DriverController = new CommandXboxController(ControllerConstants.DriverControllerID);
    private final CommandXboxController ManipulatorXboxController = new CommandXboxController(ControllerConstants.ManipulatorXboxControllerID);
    private final CommandXboxController ManipulatorGuitarController = new CommandXboxController(ControllerConstants.ManipulatorGuitarControllerID);
    private final CommandXboxController TestingController = new CommandXboxController(ControllerConstants.TestingControllerID);

    private final SendableChooser<Alliance> AllianceSelecter = new SendableChooser<>();
    private final SendableChooser<ManipulatorControllerType> ManipulatorControllerSelecter = new SendableChooser<>();
    
    private boolean driverLastLeftBumper = DriverController.leftBumper().getAsBoolean();
    private boolean driverLastRightTrigger = DriverController.leftTrigger().getAsBoolean();

    public AutoAim autoAim = new AutoAim();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();

    private Timer agitateTimer = new Timer();

    public double commandedMoveX = 0.00;
    public double commandedMoveY = 0.00;
    public double commandedRotate = 0.00;
    public double hubPIDOutput = 0.00;

    private boolean isTracking = false;
    public boolean isAutoTracking = false;
    private boolean isIntakeRetracted = false;
    private boolean isIntaking = false;
    private boolean isAgitating = false;
    private boolean isSpinup = false;
    private boolean isPassing = false;
    private boolean isShooting = false;

    public ControlSub() {

        /* Driver Controls */
        DriverController.button(ControllerConstants.XboxMenuButtonID).onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric)
        );

        DriverController.button(ControllerConstants.XboxShareButtonID).whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())
        );

        DriverController.y().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle())
        );

        ManipulatorXboxController.y().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle())
        );

        // Idle while the robot is disabled.
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true)
        );

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> ControllerDrive
                .withVelocityX(commandedMoveX * maxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(commandedMoveY * maxSpeed) // Drive left with negative X (left)
                .withRotationalRate(commandedRotate * maxAngularRate)));

        /* Manipulator Controls */
        ManipulatorControllerSelecter.setDefaultOption("Guitar Controller", ManipulatorControllerType.Guitar);
        ManipulatorControllerSelecter.addOption("Xbox Controller", ManipulatorControllerType.Xbox);

        /* AutoAim Controls */
        // Should make this work on its own but idk I cant get it
        // to work with the fms
        AllianceSelecter.setDefaultOption("Blue Alliance", Alliance.Blue);
        AllianceSelecter.addOption("Red Alliance", Alliance.Red);

        agitateTimer.reset();
    }

    @Override
    public void periodic() {
        /* Driver Controls */
        // Drive = Left Stick 
        // Steer = Right Stick
        // Intake = Right Trig
        // Toggle Intake Pos = Left Bumper
        // Track = Left Trig
        // Snake Drive = A
        // GoCrazyGoStupid = B

        if (DriverController.b().getAsBoolean()) {
            commandedMoveX = MathUtil.applyDeadband(-DriverController.getLeftY(), ControllerConstants.StickDeadzone);
            commandedMoveY = MathUtil.applyDeadband(-DriverController.getLeftX(), ControllerConstants.StickDeadzone);
            commandedRotate = MathUtil.applyDeadband(-DriverController.getRightX(), ControllerConstants.StickDeadzone);
        } else {
            commandedMoveX = XSlewRateLimiter.calculate(MathUtil.applyDeadband(-DriverController.getLeftY(), ControllerConstants.StickDeadzone));
            commandedMoveY = YSlewRateLimiter.calculate(MathUtil.applyDeadband(-DriverController.getLeftX(), ControllerConstants.StickDeadzone));
            commandedRotate = RotateSlewRateLimiter.calculate(MathUtil.applyDeadband(-DriverController.getRightX(), ControllerConstants.StickDeadzone));
        }

        if (DriverStation.isTeleop()) {
            if (DriverController.leftBumper().getAsBoolean() && !driverLastLeftBumper) {
                if (isIntakeRetracted) {
                    isIntakeRetracted = false;
                } else {
                    isIntakeRetracted = true;
                }
            }

            // When we start intaking, put the intake out, however
            // we should be able to move the intake in and out while
            // intaking
            if (DriverController.rightTrigger().getAsBoolean() && !driverLastRightTrigger) {
                isIntakeRetracted = false;
            }

            isIntaking = DriverController.rightTrigger().getAsBoolean();
            if (DriverController.rightTrigger().getAsBoolean()) {
                commandedMoveX = commandedMoveX * 0.50;
                commandedMoveY = commandedMoveY * 0.50;
            }
            
            if (isTracking) {
                DriverController.setRumble(RumbleType.kBothRumble, 0.50);
            } else {
                DriverController.setRumble(RumbleType.kBothRumble, 0.00);
            }

            // Gotta warn because the robot aiming at the stands wasnt enough
            /*if (autoAim.hasLostLimelight()) {
                DriverController.setRumble(RumbleType.kRightRumble, 0.50);
            } else {
                DriverController.setRumble(RumbleType.kRightRumble, 0.00);
            }*/
        }

        /* Manipulator Controls */
        // Spinup = X
        // Pass Spinup = B
        // Shoot = Right Trig
        // Agitate = Left Bumper
        // Track = Left Trig

        // or...

        // Spinup = Red
        // Pass Spinup = Blue
        // Shoot = Strum Up/Down
        // Agitate = Green
        // Track = Yellow
        
        if (DriverStation.isTeleop() && ManipulatorControllerSelecter.getSelected() == ManipulatorControllerType.Xbox) {
            System.out.println("hello");
            isSpinup = ManipulatorXboxController.x().getAsBoolean();
            isPassing = ManipulatorXboxController.b().getAsBoolean();
            isShooting = ManipulatorXboxController.rightTrigger().getAsBoolean();
            isAgitating = ManipulatorXboxController.leftBumper().getAsBoolean();

            if (isShooting) {
                agitateTimer.start();
            } else {
                agitateTimer.stop();
                agitateTimer.reset();
            }

            if (shooter.isShooterAtSpeed()) {
                ManipulatorXboxController.setRumble(RumbleType.kBothRumble, 0.50);
            } else {
                ManipulatorXboxController.setRumble(RumbleType.kBothRumble, 0.00);
            }

            // Gotta warn because the robot aiming at the stands wasnt enough
            /*if (autoAim.hasLostLimelight()) {
                ManipulatorController.setRumble(RumbleType.kRightRumble, 0.50);
            } else {
                ManipulatorController.setRumble(RumbleType.kRightRumble, 0.00);
            }*/
        }

        if (DriverStation.isTeleop() && ManipulatorControllerSelecter.getSelected() == ManipulatorControllerType.Guitar) {
            isSpinup = ManipulatorGuitarController.button(GuitarButtons.Red).getAsBoolean();
            isPassing = ManipulatorGuitarController.button(GuitarButtons.Blue).getAsBoolean();
            isShooting = ManipulatorGuitarController.povUp().getAsBoolean() || ManipulatorGuitarController.povDown().getAsBoolean();
            isAgitating = ManipulatorGuitarController.button(GuitarButtons.Green).getAsBoolean();

            if (isShooting) {
                agitateTimer.start();
            } else {
                agitateTimer.stop();
                agitateTimer.reset();
            }
        }

        /* Testing Controls */
        // Should be used for Sys Check at events imo
        // Rollers/Feed = A
        // Shooters = B
        // All = X
        // All W/ PID = Y

        // Never fires?
        if (DriverStation.isTeleop() && TestingController.isConnected()) {
            if (TestingController.a().getAsBoolean()) {
                shooter.setShooterDumbControl(1.00, 0.50, 0.00, 0.00, 0.00);
            } else if (TestingController.b().getAsBoolean()) {
                shooter.setShooterDumbControl(0.00, 0.00, 0.30, 0.30, 0.30);
            } else if (TestingController.x().getAsBoolean()) {
                shooter.setShooterDumbControl(1.00, 0.50, 0.30, 0.30, 0.30);
            } else if (TestingController.y().getAsBoolean()) {
                shooter.setShooterState(State.Shoot, 50.00, 50.00);
            } else {
                shooter.setShooterState(State.Idle, 0.00, 0.00);
            }
        }
        
        /* Subsystem Comtrol */

        if (isAgitating) {
            intake.setIntakeState(IntakeConstants.State.Agitate);
        } else if (isIntaking) {
            if (isIntakeRetracted) {
                intake.setIntakeState(IntakeConstants.State.IntakeIn);
            } else {
                intake.setIntakeState(IntakeConstants.State.IntakeOut);
            }
        } else {
            if (isIntakeRetracted) {
                intake.setIntakeState(IntakeConstants.State.IdleIn);
            } else {
                intake.setIntakeState(IntakeConstants.State.IdleOut);
            }
        }

        if (isPassing && isShooting) {
            shooter.setShooterState(ShooterConstants.State.Shoot, 33.00, 44.00);
        } else if (isPassing) {
            shooter.setShooterState(ShooterConstants.State.Spinup, 30.00, 40.00);
        } else if (isAutoTracking) {
            if (isSpinup && isShooting) {
                shooter.setShooterState(ShooterConstants.State.Shoot, autoAim.getShootOnMoveAimTarget()[2], autoAim.getShootOnMoveAimTarget()[3]);
            } else if (isSpinup) {
                shooter.setShooterState(ShooterConstants.State.Spinup, autoAim.getShootOnMoveAimTarget()[2], autoAim.getShootOnMoveAimTarget()[3]);
            } else {
                shooter.setShooterState(State.Idle, 0.00, 0.00);
            }
        } else if (isTracking) {
            if (isSpinup && isShooting) {
                shooter.setShooterState(ShooterConstants.State.Shoot, autoAim.getShootOnMoveAimTarget()[2], autoAim.getShootOnMoveAimTarget()[3]);
            } else if (isSpinup) {
                shooter.setShooterState(ShooterConstants.State.Spinup, autoAim.getShootOnMoveAimTarget()[2], autoAim.getShootOnMoveAimTarget()[3]);
            } else {
                shooter.setShooterState(State.Idle, 0.00, 0.00);
            }
        } else {
            if (isSpinup && isShooting) {
                shooter.setShooterState(ShooterConstants.State.Shoot, 35.00, 35.00);
            } else if (isSpinup) {
                shooter.setShooterState(ShooterConstants.State.Spinup, 35.00, 35.00);
            } else {
                shooter.setShooterState(State.Idle, 0.00, 0.00);
            }
        }

        /* Auto Aim Control */

        // This works ig
        autoAim.setAutoAimDrivetrainState(drivetrain);
        
        if (DriverStation.isTeleop()) {
            isTracking = DriverController.leftTrigger().getAsBoolean() || 
                         DriverController.a().getAsBoolean() ||
                         ManipulatorXboxController.leftTrigger().getAsBoolean() || 
                         ManipulatorGuitarController.button(GuitarButtons.Yellow).getAsBoolean();
            //autoAim.setAutoAimState(AutoAimConstants.State.Goal); idk figure out a way to make the goal state not useless or sumthin
            // If driver needs tracking let them have it, else manipulator can do whatever ig
            if (DriverController.a().getAsBoolean()) {
                isTracking = true;
                if (AllianceSelecter.getSelected() == Alliance.Blue) {
                autoAim.setAutoAimDumbControl(-commandedMoveX + drivetrain.getState().Pose.getX(), -commandedMoveY + drivetrain.getState().Pose.getY()); // Snake drive!
                                                                               // Uses controller inputs instead of
                                                                               // robot speed to remove studdering
                } else {
                autoAim.setAutoAimDumbControl(commandedMoveX + drivetrain.getState().Pose.getX(), commandedMoveY + drivetrain.getState().Pose.getY()); // Snake drive!
                                                                               // Uses controller inputs instead of
                                                                               // robot speed to remove studdering
                }
            } else if (AllianceSelecter.getSelected() == Alliance.Blue) {
                autoAim.setAutoAimDumbControl(AutoAimConstants.BlueGoal.getX(), AutoAimConstants.BlueGoal.getY());
            } else {
                autoAim.setAutoAimDumbControl(AutoAimConstants.RedGoal.getX(), AutoAimConstants.RedGoal.getY());
            }

            SmartDashboard.putData("Alliance", AllianceSelecter);
            SmartDashboard.putData("Manipulator Controller", ManipulatorControllerSelecter);
        }

        // Check if we need to go further then half a rotation and if
        // we need to select a coterminal angle depending on which is least 
        if (Math.abs(drivetrain.getState().Pose.getRotation().getDegrees() - autoAim.getShootOnMoveAimTarget()[0]) > 180) {
            if (drivetrain.getState().Pose.getRotation().getDegrees() > autoAim.getShootOnMoveAimTarget()[0]) {
                hubPIDOutput = HubTrackingPidController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), autoAim.getShootOnMoveAimTarget()[0] + 360);
            } else {
                hubPIDOutput = HubTrackingPidController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), autoAim.getShootOnMoveAimTarget()[0] - 360);
            }
        } else {
            hubPIDOutput = HubTrackingPidController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), autoAim.getShootOnMoveAimTarget()[0]);
        }

        if (isTracking) {
            commandedRotate += hubPIDOutput;
        }

        /* Output */

        SmartDashboard.putNumber("Hub Tracking Pid Output", hubPIDOutput);

        /* Input Trigger Stuffs */

        // Inputs are now "outdated" and can be compared with new ones next scheduler run
        driverLastLeftBumper = DriverController.leftBumper().getAsBoolean();
        driverLastRightTrigger = DriverController.leftTrigger().getAsBoolean();
    }

    public double getTrackingOutput() {
        return hubPIDOutput;
    }

    // Auto functions
    public void stopIntakeOut() {
        isIntakeRetracted = false;
        isIntaking = false;
    }

    public void stopIntakeIn() {
        isIntakeRetracted = true;
        isIntaking = false;
    }

    public void startIntakeOut() {
        isIntakeRetracted = false;
        isIntaking = true;
    }

    public void startIntakeIn() {
        isIntakeRetracted = true;
        isIntaking = true;
    }

    public void stopSpinup() {
        isSpinup = false;
    }

    public void startSpinup() {
        isSpinup = true;
    }

    public void stopShooting() {
        isAutoTracking = false;
        isShooting = false;
        agitateTimer.stop();
        agitateTimer.reset();
        isAgitating = false;
    }

    public void startShoot() {
        isAutoTracking = true;
        isShooting = agitateTimer.get() > 0.25;
        agitateTimer.start();
        isAgitating = agitateTimer.get() > ControllerConstants.IntakeAgitateTime;
    }

    // agitate sooner because we will have less balls
    public void startShoot2ndShot() {
        isAutoTracking = true;
        isShooting = agitateTimer.get() > 0.25;
        agitateTimer.start();
        isAgitating = agitateTimer.get() > ControllerConstants.IntakeAgitateTime - 0.75;
    }
}
