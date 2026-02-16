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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.Constants.AutoAimConstants;
import frc.robot.constants.Constants.ClimberConstants.State;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.ControllerConstants.DrivetrainState;
import frc.robot.constants.Constants.ShooterConstants;
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

    private final PIDController HubTrackingPidController = new PIDController(AutoAimConstants.TrackingHubPIDkP, AutoAimConstants.TrackingHubPIDkI, AutoAimConstants.TrackingHubPIDkD);
    
    private final PIDController ClimbTrackingXPIDController = new PIDController(AutoAimConstants.TrackingClimbMovePIDkP, AutoAimConstants.TrackingClimbMovePIDkI, AutoAimConstants.TrackingClimbMovePIDkD);
    private final PIDController ClimbTrackingYPIDController = new PIDController(AutoAimConstants.TrackingClimbMovePIDkP, AutoAimConstants.TrackingClimbMovePIDkI, AutoAimConstants.TrackingClimbMovePIDkD);
    private final PIDController ClimbTrackingRotPIDController = new PIDController(AutoAimConstants.TrackingClimbRotPIDkP, AutoAimConstants.TrackingClimbRotPIDkI, AutoAimConstants.TrackingClimbRotPIDkD);

    private final CommandXboxController DriverController = new CommandXboxController(ControllerConstants.DriverControllerID);
    private final CommandXboxController ManipulatorController = new CommandXboxController(ControllerConstants.ManipulatorControllerID);
    
    private final SendableChooser<DrivetrainState> DrivetrainChooser = new SendableChooser<>();

    private boolean driverLastA = DriverController.a().getAsBoolean();
    private boolean driverLastB = DriverController.b().getAsBoolean();
    private boolean driverLastY = DriverController.y().getAsBoolean();
    private boolean driverLastPovUp = DriverController.povUp().getAsBoolean();
    private boolean driverLastPovDown = DriverController.povDown().getAsBoolean();

    private DrivetrainState drivetrainStateLastChose = DrivetrainChooser.getSelected();
    
    public AutoAim autoAim = new AutoAim();
    public Climber climber = new Climber();
    public Intake intake = new Intake();
    public Pneumatics pneumatics = new Pneumatics();
    public Shooter shooter = new Shooter();

    private double hubPIDOutput = 0.00;
    
    private double climbXPIDOutput = 0.00;
    private double climbYPIDOutput = 0.00;
    private double climbRotPIDOutput = 0.00;

    // temp vars
    private double bottomShooterSpeed = 0;
    private double topShooterSpeed = 0;
    private boolean weAreIdlingYo = true;

    public ControlSub() {

        /* Driver Controls */
        DrivetrainChooser.setDefaultOption("Disable Drivetrain", DrivetrainState.DisabledDrivetrain);
        DrivetrainChooser.addOption("Baby Movement", DrivetrainState.BabyMode);
        DrivetrainChooser.addOption("Slow Traction Control", DrivetrainState.SlowTC);
        DrivetrainChooser.addOption("Event Traction Control", DrivetrainState.EventTC);
        DrivetrainChooser.addOption("Defence Mode (Go Crazy)", DrivetrainState.GoCrazyGoStupid);
        
        SmartDashboard.putData("Drivetrain Mode", DrivetrainChooser);

        drivetrainApplyRequest(DrivetrainChooser.getSelected());

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

        if (drivetrainStateLastChose != DrivetrainChooser.getSelected()) {
            drivetrainApplyRequest(DrivetrainChooser.getSelected());
        }

        // temp buttons
        if (!driverLastA && DriverController.a().getAsBoolean()) {
            bottomShooterSpeed = bottomShooterSpeed - 5.00;
        }

        if (!driverLastY && DriverController.y().getAsBoolean()) {
            bottomShooterSpeed = bottomShooterSpeed + 5.00;
        }

        if (!driverLastPovDown && DriverController.povDown().getAsBoolean()) {
            topShooterSpeed = topShooterSpeed - 0.10;
        }

        if (!driverLastPovUp && DriverController.povUp().getAsBoolean()) {
            topShooterSpeed = topShooterSpeed + 0.10;
        }

        // "reset/idle" button
        if (!driverLastB && DriverController.b().getAsBoolean()) {
            if (weAreIdlingYo) {
                weAreIdlingYo = false;
            } else {
                weAreIdlingYo = true;
            }
        }

        if (weAreIdlingYo) {
            shooter.setShooterState(ShooterConstants.State.Idle, 0, 0);
            bottomShooterSpeed = 0;
            topShooterSpeed = 0;
        } else {
            shooter.setShooterState(ShooterConstants.State.Shoot, bottomShooterSpeed, topShooterSpeed);
        }

        if (DriverController.povLeft().getAsBoolean()) {
            climber.setClimberDumbControl(0.40);
        } else if (DriverController.povRight().getAsBoolean()) {
            climber.setClimberDumbControl(-0.40);
        } else {
            climber.setClimberState(State.Idle);
        }

        /* Manipulator Controls */

        if (shooter.isShooterAtSpeed()) {
            ManipulatorController.setRumble(RumbleType.kBothRumble, 0.10);
        } else {
            ManipulatorController.setRumble(RumbleType.kBothRumble, 0.00);
        }

        /* Auto Aim Control */

        autoAim.setAutoAimDrivetrainState(drivetrain);

        hubPIDOutput = HubTrackingPidController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), autoAim.getShootOnMoveAimTarget()[0]);

        climbXPIDOutput = ClimbTrackingXPIDController.calculate(drivetrain.getState().Pose.getX(), autoAim.getClimbDistance()[0]);
        climbYPIDOutput = ClimbTrackingYPIDController.calculate(drivetrain.getState().Pose.getY(), autoAim.getClimbDistance()[1]);
        
        // Fix rot setpoints because I may have it backwards
        if (autoAim.isBlue()) {
            climbRotPIDOutput = ClimbTrackingRotPIDController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), 0.00);
        } else {
            climbRotPIDOutput = ClimbTrackingRotPIDController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), 180.00);
        }

        /* Output */

        SmartDashboard.putBoolean("Idle", weAreIdlingYo);

        SmartDashboard.putNumber("Hub Tracking Pid Output", hubPIDOutput);

        SmartDashboard.putNumber("Climb Tracking X Pid Output", climbXPIDOutput);
        SmartDashboard.putNumber("Climb Tracking Y Pid Output", climbYPIDOutput);
        SmartDashboard.putNumber("Climb Tracking Rot Pid Output", climbRotPIDOutput);

        // Inputs are now "outdated" and can be compared with new ones next scheduler run
        driverLastA = DriverController.a().getAsBoolean();
        driverLastB = DriverController.b().getAsBoolean();
        driverLastY = DriverController.y().getAsBoolean();
        driverLastPovUp = DriverController.povUp().getAsBoolean();
        driverLastPovDown = DriverController.povDown().getAsBoolean();

        drivetrainStateLastChose = DrivetrainChooser.getSelected();
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
        // Setting default command has drivetrain run set request periodically
        System.out.println("Setting new drivetrain request");

        drivetrain.removeDefaultCommand();
        switch (stateToChangeTo) {
            case DisabledDrivetrain:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> ControllerDrive
                        .withVelocityX(0.00) // Drive forward with negative Y (forward)
                        .withVelocityY(0.00) // Drive left with negative X (left)
                        .withRotationalRate(0.00) // Drive counterclockwise with negative X (left)
                    )
                );
            case BabyMode:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> ControllerDrive
                        .withVelocityX(
                            MathUtil.applyDeadband(-DriverController.getLeftY() * (maxSpeed / 6.00), ControllerConstants.StickDeadzone)) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            MathUtil.applyDeadband(-DriverController.getLeftX() * (maxSpeed / 6.00), ControllerConstants.StickDeadzone)) // Drive left with negative X (left)
                        .withRotationalRate(
                            MathUtil.applyDeadband(-DriverController.getRightX() * maxAngularRate, ControllerConstants.StickDeadzone)) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
            case SlowTC:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> ControllerDrive
                        .withVelocityX(
                            MathUtil.applyDeadband(
                                XSlewRateLimiter.calculate(-DriverController.getLeftY() * (maxSpeed / 2.50)), ControllerConstants.StickDeadzone)) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            MathUtil.applyDeadband(
                                YSlewRateLimiter.calculate(-DriverController.getLeftX() * (maxSpeed / 2.50)), ControllerConstants.StickDeadzone)) // Drive left with negative X (left)
                        .withRotationalRate(
                            MathUtil.applyDeadband(
                                RotateSlewRateLimiter.calculate(-DriverController.getRightX() * maxAngularRate), ControllerConstants.StickDeadzone)) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
            case EventTC:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> ControllerDrive
                        .withVelocityX(
                            MathUtil.applyDeadband(
                                XSlewRateLimiter.calculate(-DriverController.getLeftY() * maxSpeed), ControllerConstants.StickDeadzone)) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            MathUtil.applyDeadband(
                                YSlewRateLimiter.calculate(-DriverController.getLeftX() * maxSpeed), ControllerConstants.StickDeadzone)) // Drive left with negative X (left)
                        .withRotationalRate(
                            MathUtil.applyDeadband(
                                RotateSlewRateLimiter.calculate(-DriverController.getRightX() * maxAngularRate), ControllerConstants.StickDeadzone)) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
            case GoCrazyGoStupid:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> ControllerDrive
                        .withVelocityX(
                            MathUtil.applyDeadband(-DriverController.getLeftY() * maxSpeed, ControllerConstants.StickDeadzone)) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            MathUtil.applyDeadband(-DriverController.getLeftX() * maxSpeed, ControllerConstants.StickDeadzone)) // Drive left with negative X (left)
                        .withRotationalRate(
                            MathUtil.applyDeadband(-DriverController.getRightX() * maxAngularRate, ControllerConstants.StickDeadzone)) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
            case HubTracking:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> TrackDrive
                        .withVelocityX(
                            MathUtil.applyDeadband(
                                XSlewRateLimiter.calculate(-DriverController.getLeftY() * maxSpeed), ControllerConstants.StickDeadzone)) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            MathUtil.applyDeadband(
                                YSlewRateLimiter.calculate(-DriverController.getLeftX() * maxSpeed), ControllerConstants.StickDeadzone)) // Drive left with negative X (left)
                        .withRotationalRate(
                            MathUtil.clamp(hubPIDOutput * maxSpeed, -1.00, 1.00)) // Drive counterclockwise with negative X (left)
                    )
                );
                break;
            case ClimbTracking:
                drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> TrackDrive
                        .withVelocityX(
                            MathUtil.clamp(climbXPIDOutput * maxSpeed, -1.00, 1.00)) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            MathUtil.clamp(climbYPIDOutput * maxSpeed, -1.00, 1.00)) // Drive left with negative X (left)
                        .withRotationalRate(
                            MathUtil.clamp(climbRotPIDOutput * maxSpeed, -1.00, 1.00)) // Drive counterclockwise with negative X (left)
                    )
                );
        }
    }
}
