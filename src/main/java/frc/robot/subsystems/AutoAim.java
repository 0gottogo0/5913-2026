// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.AutoAimConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    Pose2d adjustedGoalPose = new Pose2d();

    PoseEstimate LimelightCenterMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightCenter);
    PoseEstimate LimelightRightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightRight);
    PoseEstimate LimelightClimbMessurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightClimb);

    double calculatedShot[] = {0.00, 0.00, 0.00, 0.00, 0.00};
    double distanceFromClimb[] = {0.00, 0.00};

    State state = State.Goal;

    public AutoAim() {
        
    }

    @Override
    public void periodic() {

        if (DriverStation.isDisabled()) {
            NetworkTableInstance.getDefault().getTable(LimelightCenter).getEntry("pipeline").setNumber(1);
            NetworkTableInstance.getDefault().getTable(LimelightRight).getEntry("pipeline").setNumber(1);
        } else {
            NetworkTableInstance.getDefault().getTable(LimelightCenter).getEntry("pipeline").setNumber(0);
            NetworkTableInstance.getDefault().getTable(LimelightRight).getEntry("pipeline").setNumber(0);
        }

        LimelightHelpers.SetRobotOrientation(LimelightClimb, robotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        LimelightCenterMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightCenter);
        LimelightRightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightRight);
        LimelightClimbMessurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightClimb);

        if (LimelightCenterMeasurement != null && LimelightCenterMeasurement.tagCount > 0) {
            drivetrain.addVisionMeasurement(LimelightCenterMeasurement.pose, LimelightCenterMeasurement.timestampSeconds);
        }

        if (LimelightRightMeasurement != null && LimelightRightMeasurement.tagCount > 0) {
            drivetrain.addVisionMeasurement(LimelightRightMeasurement.pose, LimelightRightMeasurement.timestampSeconds);
        }

        if (LimelightClimbMessurement != null && LimelightClimbMessurement.tagCount > 0 && robotSpeed.omegaRadiansPerSecond < 2) {
            drivetrain.addVisionMeasurement(LimelightClimbMessurement.pose, LimelightClimbMessurement.timestampSeconds);
        }

        switch (state) {
            case Goal:
                // Try catch statement because we might not be
                // connected to driverstation
                if (isBlue()) {
                    goalPose = BlueGoal;
                } else {
                    goalPose = RedGoal;
                }
                break;
            case NeutralZone:
                goalPose = NeutralZone;
                break;
            case AllianceZone:
                if (isBlue()) {
                    goalPose = BlueZone;
                } else {
                    goalPose = RedZone;
                }
                break;
            case DumbControl:
                goalPose = new Pose2d(0, 0, new Rotation2d(0));
                break;
            case ClimbLeft:
                if (isBlue()) {
                    goalPose = BlueClimbLeft;
                } else {
                    goalPose = RedClimbLeft;
                }
                break;
            case ClimbRight:
                if (isBlue()) {
                    goalPose = BlueClimbRight;
                } else {
                    goalPose = RedClimbRight;
                }
                break;
        }

        //TurretRotatePointPose = robotPose.plus(TurretRotatePoint);

        // To adjust the goal pose when shooting on the
        // move, first take robot speed and times that
        // by our time of flight, then add that new
        // transform to the goal pose
        adjustedGoalPose = goalPose;//.plus(new Transform2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond, new Rotation2d()).times(TimeOfFlightByDistance.get(getAimTargetInDistance())));

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
        SmartDashboard.putNumber("Target Time Of Flight", TimeOfFlightByDistance.get(getAimTargetInDistance()));
        
        SmartDashboard.putNumber("Adjusted Target Angle", getShootOnMoveAimTarget()[0]);
        SmartDashboard.putNumber("Adjusted Target Distance", getShootOnMoveAimTarget()[1]);
        SmartDashboard.putNumber("Adjusted Target Bottom Shooter Speed", getShootOnMoveAimTarget()[2]);
        SmartDashboard.putNumber("Adjusted Target Top Shooter Speed", getShootOnMoveAimTarget()[3]);
        SmartDashboard.putNumber("Adjusted Target Time Of Flight", getShootOnMoveAimTarget()[4]);
        SmartDashboard.putNumberArray("Adjusted Target Data", getShootOnMoveAimTarget());

        SmartDashboard.putNumber("Distance From Climb X", getClimbDistance()[0]);
        SmartDashboard.putNumber("Distance From Climb Y", getClimbDistance()[1]);
        SmartDashboard.putNumberArray("Distance From Climb Data", getClimbDistance());
    }

    /**
     * Give the auto aim subsystem the robots state
     * 
     * @param drivetrainToGive The robots drivetrain
     */
    public void setAutoAimDrivetrainState(CommandSwerveDrivetrain drivetrainToGive) {
        drivetrain = drivetrainToGive;
        robotPose = drivetrain.getState().Pose;
        robotSpeed = drivetrain.getState().Speeds;
    }

    /**
     * Are we on the blue aliance
     * 
     * @return true if on blue aliance
     */
    public boolean isBlue() {
        try {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                return true;
            } else {
                return false;
            }
        } catch (Exception e) {
            return true;
        }
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
        //calculatedShot[0] = Math.toDegrees(Math.atan2(robotPose.getX() - adjustedGoalPose.getX(), robotPose.getY() - adjustedGoalPose.getY()));
        calculatedShot[0] = Math.toDegrees(Math.atan2(robotPose.getX() - adjustedGoalPose.getX(), robotPose.getY() - adjustedGoalPose.getY()));
        // Gets distance for turret
        //calculatedShot[1] =Math.sqrt(Math.pow(Math.abs(TurretRotatePointPose.getX() - adjustedGoalPose.getX()), 2) + Math.pow(Math.abs(TurretRotatePointPose.getY() - adjustedGoalPose.getY()), 2));
        // Gets distance for no turret
        calculatedShot[1] = Math.sqrt(Math.pow(Math.abs(TurretRotatePointPose.getX() - adjustedGoalPose.getX()), 2) + Math.pow(Math.abs(TurretRotatePointPose.getY() - adjustedGoalPose.getY()), 2));
        // Interpolates for top shooter
        calculatedShot[2] = TopShooterSpeedByDistance.get(calculatedShot[1]);
        // Interpolates for hood shooter
        calculatedShot[3] = HoodShooterSpeedByDistance.get(calculatedShot[1]);
        // Interpolates time of flight
        calculatedShot[4] = TimeOfFlightByDistance.get(calculatedShot[1]);
        return calculatedShot;
    }

    /**
     * Gets the distance from the climb
     * 
     * @return An Array with an X and Y
     */    
    public double[] getClimbDistance() {
        distanceFromClimb[0] = robotPose.getX() - goalPose.getX();
        distanceFromClimb[1] = robotPose.getY() - goalPose.getY();
        return distanceFromClimb;
    }

    /**
	 * Sets the state to autoaim too.
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
     * Unimplimented
     */
    public void setAutoAimDumbControl() {

    }
}
