// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants.Controllers;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.FeederConstants;
import frc.robot.constants.TunerConstants;

public class ControlSub extends SubsystemBase {

    private double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double maxAngularRate = RotationsPerSecond.of(Controllers.RotateMagnitude).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    private final SwerveRequest.FieldCentric Controllerdrive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * Controllers.StickDeadzone).withRotationalDeadband(maxAngularRate * Controllers.StickDeadzone) // Add deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public AutoAim autoAim = new AutoAim();
    public Feeder feeder = new Feeder();
    public Shooter shooter = new Shooter();
    
    private final CommandXboxController DriverController = new CommandXboxController(Controllers.DriverControllerID);
    private final CommandXboxController ManipulatorController = new CommandXboxController(Controllers.ManipulatorControllerID);

    private boolean driverLastA = DriverController.a().getAsBoolean();
    private boolean driverLastB = DriverController.b().getAsBoolean();
    private boolean driverLastY = DriverController.y().getAsBoolean();
    private boolean driverLastPovUp = DriverController.povUp().getAsBoolean();
    private boolean driverLastPovDown = DriverController.povDown().getAsBoolean();

    // temp vars
    private double shooterSpeed = 0;
    private boolean weAreIdlingYo = true;

    public ControlSub() {
        drivetrainApplyRequest();
    }

    @Override
    public void periodic() {
        // Check for input then call subsystems

        /* Driver Controls */
        
        // Apply neutral mode while disabled.
        if (DriverStation.isDisabled()) {
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true);
        }

        // Set Field Centric
        if (DriverController.button(7).getAsBoolean()) {
            drivetrain.seedFieldCentric();
        }

        // Stop Swerve
        if (DriverController.button(8).getAsBoolean()) {
            drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
        }

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

        if (shooter.isShooterAtSpeed()) {
            DriverController.setRumble(RumbleType.kBothRumble, 0.10);
        } else {
            DriverController.setRumble(RumbleType.kBothRumble, 0.00);
        }

        /* Manipulator Controls */

        if (shooter.isShooterAtSpeed()) {
            ManipulatorController.setRumble(RumbleType.kBothRumble, 0.10);
        } else {
            ManipulatorController.setRumble(RumbleType.kBothRumble, 0.00);
        }

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
                Controllerdrive.withVelocityX(-DriverController.getLeftY() * maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-DriverController.getLeftX() * maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-DriverController.getRightX() * maxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    }
}
