// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.FeederConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.ClimberConstants.State;
import frc.robot.constants.TunerConstants;

public class ControlSub extends SubsystemBase {

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double maxAngularRate = RotationsPerSecond.of(ControllerConstants.RotateMagnitude).in(RadiansPerSecond);
    
    // Drive with controller request
    private final SwerveRequest.FieldCentric Controllerdrive = new SwerveRequest.FieldCentric()
            //.withDeadband(maxSpeed * Controllers.StickDeadzone).withRotationalDeadband(maxAngularRate * Controllers.StickDeadzone)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Stop request
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

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
    public Shooter shooter = new Shooter();

    // temp vars
    private double shooterSpeed = 0;
    private boolean weAreIdlingYo = true;

    public ControlSub() {

        /* Driver Controls */
        drivetrainApplyRequest();

        DriverController.button(7).onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric)
        );

        DriverController.button(8).whileTrue(
            drivetrain.applyRequest(() -> brake)
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
            shooterSpeed = shooterSpeed - 1;
        }

        if (!driverLastY && DriverController.y().getAsBoolean()) {
            shooterSpeed = shooterSpeed + 1;
        }

        if (!driverLastPovDown && DriverController.povDown().getAsBoolean()) {
            shooterSpeed = shooterSpeed - 25;
        }

        if (!driverLastPovUp && DriverController.povUp().getAsBoolean()) {
            shooterSpeed = shooterSpeed + 25;
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
            shooter.setShooterState(ShooterConstants.State.Idle, 0);
            shooterSpeed = 0;
        } else if (DriverController.x().getAsBoolean()) {
            feeder.setFeederState(FeederConstants.State.Feed, shooterSpeed);
            shooter.setShooterState(ShooterConstants.State.Shoot, shooterSpeed);
        } else {
            feeder.setFeederState(FeederConstants.State.Idle, shooterSpeed);
            shooter.setShooterState(ShooterConstants.State.Spinup, shooterSpeed);
        }

        if (DriverController.povLeft().getAsBoolean()) {
            climber.setElevatorDumbControl(0.40);
        } else if (DriverController.povRight().getAsBoolean()) {
            climber.setElevatorDumbControl(-0.40);
        } else {
            climber.setElevatorState(State.Idle, 0.00);
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
     * Apply drivetrain request
     * <p>
     * Uses setDefaultCommand() so do not call this function
     * every scheduler run.
     */
    private void drivetrainApplyRequest() {
        drivetrain.removeDefaultCommand();
        
        // Add different "modes" (tracking and stuff)
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                Controllerdrive.withVelocityX(MathUtil.applyDeadband((-DriverController.getLeftY() * maxSpeed), ControllerConstants.StickDeadzone)) // Drive forward with negative Y (forward)
                               .withVelocityY(MathUtil.applyDeadband(-DriverController.getLeftX() * maxSpeed, ControllerConstants.StickDeadzone)) // Drive left with negative X (left)
                               .withRotationalRate(MathUtil.applyDeadband(-DriverController.getRightX() * maxAngularRate, ControllerConstants.StickDeadzone)) // Drive counterclockwise with negative X (left)
            )
        );
    }
}
