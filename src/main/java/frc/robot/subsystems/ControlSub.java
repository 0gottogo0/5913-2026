// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.FeederConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.ClimberConstants.State;
import frc.robot.constants.Constants.ControllerConstants.DrivetrainState;
import frc.robot.constants.TunerConstants;

public class ControlSub extends SubsystemBase {

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double maxAngularRate = RotationsPerSecond.of(ControllerConstants.RotateMagnitude).in(RadiansPerSecond);
    
    // Drive with controller request
    private final SwerveRequest.FieldCentric ControllerDrive = new SwerveRequest.FieldCentric()
        //.withDeadband(maxSpeed * Controllers.StickDeadzone).withRotationalDeadband(maxAngularRate * Controllers.StickDeadzone)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric TrackDrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SlewRateLimiter XSlewRateLimiter = new SlewRateLimiter(ControllerConstants.XSlewRateLimiter);
    private final SlewRateLimiter YSlewRateLimiter = new SlewRateLimiter(ControllerConstants.YSlewRateLimiter);
    private final SlewRateLimiter RotateSlewRateLimiter = new SlewRateLimiter(ControllerConstants.RotateSlewRateLimiter);

    private final CommandXboxController DriverController = new CommandXboxController(ControllerConstants.DriverControllerID);
    private final CommandXboxController ManipulatorController = new CommandXboxController(ControllerConstants.ManipulatorControllerID);
    
    private boolean driverLastA = DriverController.a().getAsBoolean();
    private boolean driverLastB = DriverController.b().getAsBoolean();
    private boolean driverLastY = DriverController.y().getAsBoolean();
    private boolean driverLastPovUp = DriverController.povUp().getAsBoolean();
    private boolean driverLastPovDown = DriverController.povDown().getAsBoolean();
    
    public AutoAim autoAim = new AutoAim();
    public Climber climber = new Climber();
    public Feeder feeder = new Feeder();
    public Intake intake = new Intake();
    public Pneumatics pneumatics = new Pneumatics();
    public Shooter shooter = new Shooter();

    // temp vars
    private double bottomShooterSpeed = 0;
    private double topShooterSpeed = 0;
    private boolean weAreIdlingYo = true;

    public DrivetrainState drivetrainState = DrivetrainState.EventTC;

    public ControlSub() {

        /* Driver Controls */
        drivetrainApplyRequest(DrivetrainState.EventTC);

        DriverController.button(ControllerConstants.XboxMenuButtonID).onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric)
        );

        DriverController.button(ControllerConstants.XboxShareButtonID).whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())
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

        // temp buttons
        if (!driverLastA && DriverController.a().getAsBoolean()) {
            bottomShooterSpeed = bottomShooterSpeed - 5;
        }

        if (!driverLastY && DriverController.y().getAsBoolean()) {
            bottomShooterSpeed = bottomShooterSpeed + 5;
        }

        if (!driverLastPovDown && DriverController.povDown().getAsBoolean()) {
            topShooterSpeed = topShooterSpeed - 5;
        }

        if (!driverLastPovUp && DriverController.povUp().getAsBoolean()) {
            topShooterSpeed = topShooterSpeed + 5;
        }

        // "reset" button
        if (!driverLastB && DriverController.b().getAsBoolean()) {
            if (weAreIdlingYo) {
                weAreIdlingYo = false;
            } else {
                weAreIdlingYo = true;
            }
        }

        if (weAreIdlingYo) {
            feeder.setFeederState(FeederConstants.State.Idle, 0);
            shooter.setShooterState(ShooterConstants.State.Idle, 0, 0);
            bottomShooterSpeed = 0;
            topShooterSpeed = 0;
        } else if (DriverController.x().getAsBoolean()) {
            feeder.setFeederState(FeederConstants.State.Feed, bottomShooterSpeed);
            shooter.setShooterState(ShooterConstants.State.Shoot, bottomShooterSpeed, topShooterSpeed);
        } else {
            feeder.setFeederState(FeederConstants.State.Idle, bottomShooterSpeed);
            shooter.setShooterState(ShooterConstants.State.Spinup, bottomShooterSpeed, topShooterSpeed);
        }

        if (DriverController.povLeft().getAsBoolean()) {
            climber.setElevatorDumbControl(0.40, false);
        } else if (DriverController.povRight().getAsBoolean()) {
            climber.setElevatorDumbControl(-0.40, false);
        } else {
            climber.setElevatorState(State.Idle);
        }

        /* Manipulator Controls */

        if (shooter.isShooterAtSpeed()) {
            ManipulatorController.setRumble(RumbleType.kBothRumble, 0.10);
        } else {
            ManipulatorController.setRumble(RumbleType.kBothRumble, 0.00);
        }

        /* Auto Aim Control */

        autoAim.setAutoAimDrivetrainState(drivetrain.getState().Pose, drivetrain.getState().Speeds);

        /* Output */

        SmartDashboard.putBoolean("Idle", weAreIdlingYo);

        // Inputs are now "outdated" and can be compared with new ones next scheduler run
        driverLastA = DriverController.a().getAsBoolean();
        driverLastB = DriverController.b().getAsBoolean();
        driverLastY = DriverController.y().getAsBoolean();
        driverLastPovUp = DriverController.povUp().getAsBoolean();
        driverLastPovDown = DriverController.povDown().getAsBoolean();

    }

    /**
     * Apply drivetrain request with state
     * <p>
     * Uses setDefaultCommand() so do not call this function
     * every scheduler run.
     * 
     * @param stateToChangeTo State to change the drivetrain to
     */
    private void drivetrainApplyRequest(DrivetrainState stateToChangeTo) {
        // Setting default command has drivetrain run request periodically
        drivetrain.removeDefaultCommand();
        switch (stateToChangeTo) {
            case DisabledDrivetrain:
                // WHERES THE VAN
                // THEY SAID IT WAS GONNA BE HERE
            case BabyMode:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> ControllerDrive
                        .withVelocityX(MathUtil.applyDeadband(-DriverController.getLeftY() * (maxSpeed / 6), ControllerConstants.StickDeadzone)) // Drive forward with negative Y (forward)
                        .withVelocityY(MathUtil.applyDeadband(-DriverController.getLeftX() * (maxSpeed / 6), ControllerConstants.StickDeadzone)) // Drive left with negative X (left)
                        .withRotationalRate(MathUtil.applyDeadband(-DriverController.getRightX() * maxAngularRate, ControllerConstants.StickDeadzone)) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
            case SlowTC:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> ControllerDrive
                        .withVelocityX(XSlewRateLimiter.calculate(MathUtil.applyDeadband(-DriverController.getLeftY() * (maxSpeed / 2), ControllerConstants.StickDeadzone))) // Drive forward with negative Y (forward)
                        .withVelocityY(YSlewRateLimiter.calculate(MathUtil.applyDeadband(-DriverController.getLeftX() * (maxSpeed / 2), ControllerConstants.StickDeadzone))) // Drive left with negative X (left)
                        .withRotationalRate(RotateSlewRateLimiter.calculate(MathUtil.applyDeadband(-DriverController.getRightX() * maxAngularRate, ControllerConstants.StickDeadzone))) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
            case EventTC:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> ControllerDrive
                        .withVelocityX(XSlewRateLimiter.calculate(MathUtil.applyDeadband(-DriverController.getLeftY() * maxSpeed, ControllerConstants.StickDeadzone))) // Drive forward with negative Y (forward)
                        .withVelocityY(YSlewRateLimiter.calculate(MathUtil.applyDeadband(-DriverController.getLeftX() * maxSpeed, ControllerConstants.StickDeadzone))) // Drive left with negative X (left)
                        .withRotationalRate(RotateSlewRateLimiter.calculate(MathUtil.applyDeadband(-DriverController.getRightX() * maxAngularRate, ControllerConstants.StickDeadzone))) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
            case GoCrazyGoStupid:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> ControllerDrive
                        .withVelocityX(MathUtil.applyDeadband(-DriverController.getLeftY() * maxSpeed, ControllerConstants.StickDeadzone)) // Drive forward with negative Y (forward)
                        .withVelocityY(MathUtil.applyDeadband(-DriverController.getLeftX() * maxSpeed, ControllerConstants.StickDeadzone)) // Drive left with negative X (left)
                        .withRotationalRate(MathUtil.applyDeadband(-DriverController.getRightX() * maxAngularRate, ControllerConstants.StickDeadzone)) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
            case TrackingTemplate:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> TrackDrive
                        .withVelocityX(0.50 * maxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(0.50 * maxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(0.50 * maxSpeed) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
        }
    }
}
