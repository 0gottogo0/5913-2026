// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.AutoAimConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.Constants.AutoAimConstants.State;

public class AutoAim extends SubsystemBase {

    CommandSwerveDrivetrain drivetrain;
    Pose2d robotPose = new Pose2d();
    ChassisSpeeds robotSpeed = new ChassisSpeeds();

    Pose2d TurretRotatePointPose = new Pose2d();

    Pose2d goalPose = new Pose2d();
    Pose2d redGoalPose = new Pose2d();
    Pose2d adjustedGoalPose = new Pose2d();

    PoseEstimate LimelightCenterMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightCenter);

    State state = State.Goal;
    
    Field2d field = new Field2d();
    
    double calculatedShot[] = {0.00, 0.00, 0.00, 0.00, 0.00};
    double distanceFromClimb[] = {0.00, 0.00};

    boolean lostLimelight = false;

    public AutoAim() {
        
    }

    @Override
    public void periodic() {

        // Change pipeline depending on robot state
        if (DriverStation.isDisabled()) {
            NetworkTableInstance.getDefault().getTable(LimelightCenter).getEntry("pipeline").setNumber(1);
        } else {
            NetworkTableInstance.getDefault().getTable(LimelightCenter).getEntry("pipeline").setNumber(0);
        }

        LimelightCenterMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightCenter);

        if (LimelightCenterMeasurement != null && LimelightCenterMeasurement.tagCount > 0) {
            drivetrain.addVisionMeasurement(LimelightCenterMeasurement.pose, LimelightCenterMeasurement.timestampSeconds);
        }

        switch (state) {
            case Goal:
                if (isBlue()) {
                    goalPose = BlueGoal;
                } else {
                    goalPose = RedGoal;
                }
                break;
            case SnakeDrive:
                // Brokie
            case DumbControl:
                // Do nothing, handled by setAutoAimDumbControl()
                break;
        }

        // Only for turret
        //TurretRotatePointPose = robotPose.plus(TurretRotatePoint);

        // To adjust the goal pose when shooting on the
        // move, first take robot speed and times that
        // by our time of flight, then add that new
        // transform to the goal pose
        adjustedGoalPose = goalPose;//.plus(new Transform2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond, new Rotation2d()).times(TimeOfFlightByDistance.get(getAimTargetInDistance())));
        // we arnt shooting on the move so I commented out a ton of shi lol

        SmartDashboard.putNumber("Robot X", robotPose.getX());
        SmartDashboard.putNumber("Robot Y", robotPose.getY());
        SmartDashboard.putNumber("Robot Rot", robotPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Robot Speed X", robotSpeed.vxMetersPerSecond);
        SmartDashboard.putNumber("Robot Speed Y", robotSpeed.vyMetersPerSecond);
        SmartDashboard.putNumber("Robot Speed Rot", Math.toDegrees(robotSpeed.omegaRadiansPerSecond));
        
        // These are useless if we dont have a turret ;-;
        SmartDashboard.putNumber("Turret X", TurretRotatePointPose.getX());
        SmartDashboard.putNumber("Turret Y", TurretRotatePointPose.getY());
        
        SmartDashboard.putNumber("Target Angle", getAimTargetInDegrees());
        SmartDashboard.putNumber("Target Distance", getAimTargetInDistance());
        SmartDashboard.putNumber("Target Time Of Flight", 0);//TimeOfFlightByDistance.get(getAimTargetInDistance()));
        
        SmartDashboard.putNumber("Adjusted Target Angle", getShootOnMoveAimTarget()[0]);
        SmartDashboard.putNumber("Adjusted Target Distance", getShootOnMoveAimTarget()[1]);
        SmartDashboard.putNumber("Adjusted Target Bottom Shooter Speed", getShootOnMoveAimTarget()[2]);
        SmartDashboard.putNumber("Adjusted Target Top Shooter Speed", getShootOnMoveAimTarget()[3]);
        SmartDashboard.putNumber("Adjusted Target Time Of Flight", getShootOnMoveAimTarget()[4]);
        SmartDashboard.putNumberArray("Adjusted Target Data", getShootOnMoveAimTarget());

        SmartDashboard.putData("Field", field);

        // For visualization of target, NOTE: the target is
        // above the ground and balls will not land on the 
        // target, but they will pass through the target
        field.setRobotPose(robotPose);

        if (LimelightCenterMeasurement == null) {
            DataLogManager.log("Reef Camera Lost!");
            lostLimelight = true;
        } else {
            lostLimelight = false;
        }
    }

    /**
     * Give the auto aim subsystem the robots state
     * 
     * @param drivetrainToSet The robots drivetrain
     */
    public void setAutoAimDrivetrainState(CommandSwerveDrivetrain drivetrainToSet) {
        drivetrain = drivetrainToSet;
        robotPose = drivetrain.getState().Pose;
        robotSpeed = drivetrain.getState().Speeds;
    }

    /**
     * Are we on the blue aliance
     * 
     * @return true if on blue aliance
     */
    public boolean isBlue() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
        }
        return false;
    }

    /** Vro we lost tracking :(
     * 
     * @return true if limelight has disconnected (this gets logged via WPI anyways but good for driver feedback or whatever)
     */
    public boolean hasLostLimelight() {
        return lostLimelight;
    }

    /**
     * Get angle from robot to target
     * 
     * @return Aim tagret in degrees
     */
    private double getAimTargetInDegrees() {
        return robotPose.getRotation().getDegrees() - Math.atan(TurretRotatePointPose.minus(goalPose).getX() / TurretRotatePointPose.minus(goalPose).getY());
    }
    
    /**
     * Get distance from robot to target
     * 
     * @return Aim target in meters
     */
    private double getAimTargetInDistance() {
        return Math.abs(Math.sqrt(Math.pow(TurretRotatePointPose.minus(goalPose).getX(), 2) + Math.pow(TurretRotatePointPose.minus(goalPose).getY(), 2)));
    }

    /**
     * Get calculated angle, speed, and time of
     * flight to shoot into the target on the
     * move
     * 
     * @return An array with degrees, distance in meters,
     *         top rps, hood rps, and seconds
     */
    public double[] getShootOnMoveAimTarget() {
        // Code used for if we have a turret for degrees
        //calculatedShot[0] = robotPose.getRotation().getDegrees() - Math.atan(TurretRotatePointPose.minus(adjustedGoalPose).getX() / TurretRotatePointPose.minus(adjustedGoalPose).getY());
        // Code used for if we do not have a turret for degrees
        calculatedShot[0] = Math.toDegrees(Math.atan2((robotPose.getY() - adjustedGoalPose.getY()), (robotPose.getX() - adjustedGoalPose.getX())));
        // Gets distance
        calculatedShot[1] = Math.sqrt(Math.pow(Math.abs(robotPose.getX() - adjustedGoalPose.getX()), 2) + Math.pow(Math.abs(robotPose.getY() - adjustedGoalPose.getY()), 2));
        // Interpolates for top shooter
        calculatedShot[2] = TopShooterSpeedByDistance.get(calculatedShot[1]);
        // Interpolates for hood shooter
        calculatedShot[3] = HoodShooterSpeedByDistance.get(calculatedShot[1]);
        // Interpolates time of flight
        calculatedShot[4] = 0;//TimeOfFlightByDistance.get(calculatedShot[1]);
        return calculatedShot;
    }

    /**
	 * Sets the state or target.
	 * <p> 
	 * If wanting to control shots manualy then
     * use setAutoAimDumbControl()
	 * 
	 * @param stateToChangeTo Using AutoAimConstants.State	
	 */
    public void setAutoAimState(State stateToChangeTo) {
        state = stateToChangeTo;
    }

    /**
     * Sets cordinate positions to shoot at
     * <p>
     * Uses FRC WPIBlue cordinates, more, and better information
     * can be found in the Limelight docs under "3D Coordinate
     * Systems In Detail"
     */
    public void setAutoAimDumbControl(double x, double y) {
        goalPose = new Pose2d(x, y, new Rotation2d(0.00));
        state = State.DumbControl;
    }
}
