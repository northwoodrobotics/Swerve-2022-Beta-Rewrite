package ExternalLib.JackInTheBotLib.control;

import ExternalLib.JackInTheBotLib.math.RigidTransform2;
import ExternalLib.JackInTheBotLib.math.Vector2;
import ExternalLib.JackInTheBotLib.util.HolonomicDriveSignal;
import ExternalLib.JackInTheBotLib.util.HolonomicFeedforward;
import ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;
import edu.wpi.first.math.geometry.Pose2d;
import ExternalLib.NorthwoodLib.MathWrappers.NWPose2d;
import ExternalLib.NorthwoodLib.MathWrappers.NWRotation2d;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlanner;


public class HolonomicMotionProfiledTrajectoryFollower extends TrajectoryFollower<HolonomicDriveSignal> {
    private PidController forwardController;
    private PidController strafeController;
    private PidController rotationController;

    private HolonomicFeedforward feedforward;

    private PathPlannerState lastState = null;

    private boolean finished = false;

    public HolonomicMotionProfiledTrajectoryFollower(PidConstants translationConstants, PidConstants rotationConstants,
                                                     HolonomicFeedforward feedforward) {
        this.forwardController = new PidController(translationConstants);
        this.strafeController = new PidController(translationConstants);
        this.rotationController = new PidController(rotationConstants);
        this.rotationController.setContinuous(true);
        this.rotationController.setInputRange(0.0, 2.0 * Math.PI);

        this.feedforward = feedforward;
    }

    @Override
    protected HolonomicDriveSignal calculateDriveSignal(NWPose2d currentPose, NWTranslation2d velocity,
                                               double rotationalVelocity, PathPlannerTrajectory trajectory, double time,
                                               double dt) {
        if (time > trajectory.getTotalTimeSeconds()) {
            finished = true;
            return new HolonomicDriveSignal(NWTranslation2d.ZERO, 0.0, false);
        }

        lastState = trajectory.getState((int) time);

        NWTranslation2d segmentVelocity = (NWTranslation2d) NWTranslation2d.fromAngle((NWRotation2d) lastState.poseMeters.getRotation()).times(lastState.velocityMetersPerSecond);
        NWTranslation2d segmentAcceleration = (NWTranslation2d) NWTranslation2d.fromAngle((NWRotation2d) lastState.poseMeters.getRotation()).times(lastState.accelerationMetersPerSecondSq);

        NWTranslation2d feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);

        forwardController.setSetpoint(lastState.poseMeters.getX());
        strafeController.setSetpoint(lastState.poseMeters.getY());
        rotationController.setSetpoint(lastState.poseMeters.getRotation().getRadians());

        return new HolonomicDriveSignal(
                new NWTranslation2d(
                        forwardController.calculate(currentPose.getX(), dt) + feedforwardVector.getX(),
                        strafeController.calculate(currentPose.getY(), dt) + feedforwardVector.getY()
                ),
                rotationController.calculate(currentPose.getRotation().getRadians(), dt),
                true
        );
    }

    public PathPlannerState getLastState() {
        return lastState;
    }

    @Override
    protected boolean isFinished() {
        return finished;
    }

    @Override
    protected void reset() {
        forwardController.reset();
        strafeController.reset();
        rotationController.reset();

        finished = false;
    }
}
