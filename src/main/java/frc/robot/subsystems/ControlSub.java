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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.Constants.AutoAimConstants;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ShooterConstants.State;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.TunerConstants;

public class ControlSub extends SubsystemBase {

    // Something with our drivetrain is fundamentally backwards and
    // I dont know what it is
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double maxAngularRate = RotationsPerSecond.of(ControllerConstants.RotateMagnitude).in(RadiansPerSecond);
    
    // Drive with controller request
    private final SwerveRequest.FieldCentric ControllerDrive = new SwerveRequest.FieldCentric()
        //.withDeadband(maxSpeed * Controllers.StickDeadzone).withRotationalDeadband(maxAngularRate * Controllers.StickDeadzone)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SlewRateLimiter XSlewRateLimiter = new SlewRateLimiter(ControllerConstants.XSlewRateLimiter);
    private final SlewRateLimiter YSlewRateLimiter = new SlewRateLimiter(ControllerConstants.YSlewRateLimiter);
    private final SlewRateLimiter RotateSlewRateLimiter = new SlewRateLimiter(ControllerConstants.RotateSlewRateLimiter);

    private final PIDController HubTrackingPidController = new PIDController(AutoAimConstants.TrackingHubPIDkP, AutoAimConstants.TrackingHubPIDkI, AutoAimConstants.TrackingHubPIDkD);
    
    private final CommandXboxController DriverController = new CommandXboxController(ControllerConstants.DriverControllerID);
    private final CommandXboxController ManipulatorController = new CommandXboxController(ControllerConstants.ManipulatorControllerID);
    private final CommandXboxController TestingController = new CommandXboxController(ControllerConstants.TestingCOntrollerID);
    
    private boolean driverLastLeftBumper = DriverController.leftBumper().getAsBoolean();
    private boolean manipulatorLastLeftBumper = ManipulatorController.leftBumper().getAsBoolean();

    public AutoAim autoAim = new AutoAim();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();

    private double commandedMoveX = 0.00;
    private double commandedMoveY = 0.00;
    private double commandedRotate = 0.00;
    private double hubPIDOutput = 0.00;
    
    private boolean isTracking = false;
    private boolean isIntakeRetracted = true;
    private boolean isIntaking = false;
    private boolean isSpinup = false;
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

        ManipulatorController.y().whileTrue(
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
        // GoCrazyGoStupid

        commandedMoveX = XSlewRateLimiter.calculate(MathUtil.applyDeadband(-DriverController.getLeftY(), ControllerConstants.StickDeadzone));
        commandedMoveY = YSlewRateLimiter.calculate(MathUtil.applyDeadband(-DriverController.getLeftX(), ControllerConstants.StickDeadzone));
        commandedRotate = RotateSlewRateLimiter.calculate(MathUtil.applyDeadband(-DriverController.getRightX(), ControllerConstants.StickDeadzone));
        // Thank you team 4539 for this great
        // name idea at the 2025 NMRC Chamionship

        if (DriverStation.isTeleop()) {
            if (DriverController.leftBumper().getAsBoolean() && !driverLastLeftBumper) {
                if (isIntakeRetracted) {
                    isIntakeRetracted = false;
                } else {
                    isIntakeRetracted = true;
                }
            }

            isIntaking = DriverController.rightTrigger().getAsBoolean();

            if (isTracking) {
                DriverController.setRumble(RumbleType.kBothRumble, 0.20);
            } else {
                DriverController.setRumble(RumbleType.kBothRumble, 0.00);
            }
        }
        

        if (isIntaking) {
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

        /* Manipulator Controls */
        // Spinup = X
        // Shoot = Right Trig
        // Track = Left Trig
        // Intake = 
        // Toggle Intake Pos = Left Bumper
        // Set Hub as Target = 
        // Manual Target = 
        
        if (DriverStation.isTeleop()) {
            isSpinup = ManipulatorController.x().getAsBoolean();
            isShooting = ManipulatorController.rightTrigger().getAsBoolean();

            if (ManipulatorController.leftBumper().getAsBoolean() && !manipulatorLastLeftBumper) {
                if (isIntakeRetracted) {
                    isIntakeRetracted = false;
                } else {
                    isIntakeRetracted = true;
                }
            }

            if (shooter.isShooterAtSpeed()) {
                ManipulatorController.setRumble(RumbleType.kBothRumble, 0.20);
            } else {
                ManipulatorController.setRumble(RumbleType.kBothRumble, 0.00);
            }
        }

        // If we are tracking, do the speed interpolation too
        if (isTracking) {
            if (isSpinup && isShooting) {
                shooter.setShooterState(ShooterConstants.State.Shoot, autoAim.getShootOnMoveAimTarget()[2], autoAim.getShootOnMoveAimTarget()[3]);
            } else if (isSpinup) {
                shooter.setShooterState(ShooterConstants.State.Spinup, autoAim.getShootOnMoveAimTarget()[2], autoAim.getShootOnMoveAimTarget()[3]);
            } else {
                shooter.setShooterState(State.Idle, 0.00, 0.00);
            }
        } else {
            if (isSpinup && isShooting) {
                shooter.setShooterState(ShooterConstants.State.Shoot, 38.00 * 1.67, 10.00);
            } else if (isSpinup) {
                shooter.setShooterState(ShooterConstants.State.Spinup, 38.00 * 1.67, 10.00);
            } else {
                shooter.setShooterState(State.Idle, 0.00, 0.00);
            }
        }

        /* Testing Controls */
        // Should be used for Sys Check at events imo
        // Shoot = B

        if (TestingController.b().getAsBoolean()) {
            shooter.setShooterState(ShooterConstants.State.Shoot, 38.00 * 1.67, 10.00);
        } else {
            shooter.setShooterState(State.Idle, 0.00, 0.00);
        }

        /* Auto Aim Control */

        // This works ig
        autoAim.setAutoAimDrivetrainState(drivetrain);
        
        if (DriverStation.isTeleop()) {
            isTracking = DriverController.leftTrigger().getAsBoolean() || ManipulatorController.leftTrigger().getAsBoolean();
        }

        // Check if we need to go further then half a rotation and if
        // we do than we can add or subtract an entire rotation depending
        // on which is least 
        if (Math.abs(drivetrain.getState().Pose.getRotation().getDegrees() - autoAim.getShootOnMoveAimTarget()[0]) > 180) {
            if (drivetrain.getState().Pose.getRotation().getDegrees() > autoAim.getShootOnMoveAimTarget()[0]) {
                hubPIDOutput = HubTrackingPidController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), autoAim.getShootOnMoveAimTarget()[0] + 360);
            } else {
                hubPIDOutput = HubTrackingPidController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), autoAim.getShootOnMoveAimTarget()[0] - 360);
            }
        } else {
            hubPIDOutput = HubTrackingPidController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), autoAim.getShootOnMoveAimTarget()[0]);
        }

        if (DriverStation.isTeleop()) {
            isTracking = DriverController.leftTrigger().getAsBoolean() || ManipulatorController.leftTrigger().getAsBoolean();
        }

        if (isTracking) {
            commandedRotate += hubPIDOutput;
        }

        /* Output */

        SmartDashboard.putNumber("Hub Tracking Pid Output", hubPIDOutput);

        // Inputs are now "outdated" and can be compared with new ones next scheduler run
        driverLastLeftBumper = DriverController.leftBumper().getAsBoolean();
        manipulatorLastLeftBumper = ManipulatorController.leftBumper().getAsBoolean();
    }

    // Start using these in teleop?
    // That way we dont have to use that stupid
    // Driverstation.isTeleop() function again...
    // ...maybe

    // Atleast get rid of these variables and
    // have these functions call the subsystem
    // function calls themself
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
        isShooting = false;
    }

    public void startShooting() {
        isShooting = true;
    }

    public void stopTracking() {
        isTracking = false;
    }

    public void startTracking() {
        isTracking = true;
    }
}
