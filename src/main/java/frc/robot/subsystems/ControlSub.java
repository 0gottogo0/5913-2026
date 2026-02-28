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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.Constants.AutoAimConstants;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ControllerConstants.DrivetrainState;
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
    
    private final SendableChooser<DrivetrainState> DrivetrainChooser = new SendableChooser<>();

    private boolean driverLastLeftBumper = DriverController.leftBumper().getAsBoolean();

    private DrivetrainState drivetrainLastState = DrivetrainChooser.getSelected();
    
    public AutoAim autoAim = new AutoAim();
    public Climber climber = new Climber();
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
        DrivetrainChooser.setDefaultOption("Disable Drivetrain", DrivetrainState.DisabledDrivetrain);
        //DrivetrainChooser.addOption("Disable Drivetrain", DrivetrainState.DisabledDrivetrain);
        //DrivetrainChooser.setDefaultOption("Baby Movement", DrivetrainState.BabyMode);
        DrivetrainChooser.addOption("Baby Movement", DrivetrainState.BabyMode);
        //DrivetrainChooser.setDefaultOption("Slow Traction Control", DrivetrainState.SlowTC);
        DrivetrainChooser.addOption("Slow Traction Control", DrivetrainState.SlowTC);
        //DrivetrainChooser.setDefaultOption("Event Traction Control", DrivetrainState.EventTC);
        DrivetrainChooser.addOption("Event Traction Control", DrivetrainState.EventTC);
        //DrivetrainChooser.setDefaultOption("Defence Mode (Go Crazy)", DrivetrainState.GoCrazyGoStupid);
        DrivetrainChooser.addOption("Defence Mode (Go Crazy)", DrivetrainState.GoCrazyGoStupid);
        
        SmartDashboard.putData("Drivetrain Mode", DrivetrainChooser);

        drivetrainApplyRequest(DrivetrainChooser.getSelected());

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
    }

    @Override
    public void periodic() {
        // Check for input then call subsystems

        /* Driver Controls */
        // Drive = Left Stick 
        // Steer = Right Stick
        // Intake = Right Trig
        // Toggle Intake Pos = Left Bumper
        // Track = Left Trig

        commandedMoveX = MathUtil.applyDeadband(-DriverController.getLeftY(), ControllerConstants.StickDeadzone);
        commandedMoveY = MathUtil.applyDeadband(-DriverController.getLeftX(), ControllerConstants.StickDeadzone);
        commandedRotate = MathUtil.applyDeadband(-DriverController.getRightX(), ControllerConstants.StickDeadzone);

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

        if (drivetrainLastState != DrivetrainChooser.getSelected()) {
            drivetrainApplyRequest(DrivetrainChooser.getSelected());
        }

        /* Manipulator Controls */
        // Spinup = X
        // Shoot = Right Trig
        // Track = Left Bumper
        // Climb Up = Pov Up
        // Climb Down = Pov Down
        
        if (DriverStation.isTeleop()) {
            isSpinup = ManipulatorController.x().getAsBoolean();
            isShooting = ManipulatorController.rightTrigger().getAsBoolean();

            if (ManipulatorController.povUp().getAsBoolean()) {
                climber.setClimberDumbControl(1.00);
            } else if (ManipulatorController.povDown().getAsBoolean()) {
                climber.setClimberDumbControl(-1.00);
            } else {
                climber.setClimberDumbControl(0.00);
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
                shooter.setShooterState(ShooterConstants.State.Shoot, 38.00, 10.00);
            } else if (isSpinup) {
                shooter.setShooterState(ShooterConstants.State.Spinup, 38.00, 10.00);
            } else {
                shooter.setShooterState(State.Idle, 0.00, 0.00);
            }
        }

        /* Auto Aim Control */

        // This works ig
        autoAim.setAutoAimDrivetrainState(drivetrain);

        // We can reverse the pid so it takes the shorter way around, its
        // hard to explan in code comments but the rotation gets rolled
        // over at 180, and -180 degrees (these exact degrees dont matter).
        // If the pid is trying to go more then half a full rotation we can 
        // just reverse it, it gets a bit rough in some edge cases however
        // it works in those cases and thats all that really matters
        if (Math.abs(drivetrain.getState().Pose.getRotation().getDegrees() - autoAim.getShootOnMoveAimTarget()[0]) > 180) {
            hubPIDOutput = -HubTrackingPidController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), autoAim.getShootOnMoveAimTarget()[0]);
        } else {
            hubPIDOutput = HubTrackingPidController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), autoAim.getShootOnMoveAimTarget()[0]);
        }

        if (DriverStation.isTeleop()) {
            isTracking = DriverController.leftTrigger().getAsBoolean() || ManipulatorController.leftBumper().getAsBoolean();
        }

        if (isTracking) {
            commandedRotate += hubPIDOutput;
        }

        /* Output */

        SmartDashboard.putNumber("Hub Tracking Pid Output", hubPIDOutput);

        // Inputs are now "outdated" and can be compared with new ones next scheduler run
        driverLastLeftBumper = DriverController.leftBumper().getAsBoolean();

        drivetrainLastState = DrivetrainChooser.getSelected();
    }

    /**
     * Apply drivetrain request with state
     * <p>
     * Uses setDefaultCommand() and the robot needs
     * to be disabled to use
     * 
     * @param stateToChangeTo State to change the drivetrain to
     */
    private void drivetrainApplyRequest(DrivetrainState stateToChangeTo) {
        // Setting default command has drivetrain run set request periodically
        System.out.println("Setting new drivetrain request");

        switch (stateToChangeTo) {
            case DisabledDrivetrain:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> ControllerDrive
                        .withVelocityX(0.00) // Drive forward with negative Y (forward)
                        .withVelocityY(0.00) // Drive left with negative X (left)
                        .withRotationalRate(0.00) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
            case BabyMode:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> ControllerDrive
                        .withVelocityX(commandedMoveX * (maxSpeed / 6.00)) // Drive forward with negative Y (forward)
                        .withVelocityY(commandedMoveY * (maxSpeed / 6.00)) // Drive left with negative X (left)
                        .withRotationalRate(commandedRotate * (maxAngularRate / 4.00)) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
            case SlowTC:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> ControllerDrive
                        .withVelocityX(
                            XSlewRateLimiter.calculate(commandedMoveX) * (maxSpeed / 2.50)) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            YSlewRateLimiter.calculate(commandedMoveY) * (maxSpeed / 2.50)) // Drive left with negative X (left)
                        .withRotationalRate(
                            RotateSlewRateLimiter.calculate(commandedRotate) * maxAngularRate) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
            case EventTC:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> ControllerDrive
                        .withVelocityX(
                            XSlewRateLimiter.calculate(commandedMoveX) * maxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            YSlewRateLimiter.calculate(commandedMoveY) * maxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(
                            RotateSlewRateLimiter.calculate(commandedRotate) * maxAngularRate) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
            case GoCrazyGoStupid:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> ControllerDrive
                        .withVelocityX(commandedMoveX * maxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(commandedMoveY * maxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(commandedRotate * maxAngularRate) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
        }
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
