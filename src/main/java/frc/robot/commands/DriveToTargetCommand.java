package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import ExternalLib.JackInTheBotLib.control.Path;
import ExternalLib.JackInTheBotLib.control.SimplePathBuilder;
import ExternalLib.JackInTheBotLib.control.Trajectory;
import ExternalLib.JackInTheBotLib.math.RigidTransform2;
import ExternalLib.JackInTheBotLib.math.Rotation2;
import ExternalLib.JackInTheBotLib.math.Vector2;
import ExternalLib.JackInTheBotLib.util.Side;
import ExternalLib.NorthwoodLib.MathWrappers.NWPose2d;
import ExternalLib.NorthwoodLib.MathWrappers.NWRotation2d;
import ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import java.util.function.Supplier;

public class DriveToTargetCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final Supplier<Side> loadingStationSide;

    private boolean shouldRegeneratePaths = true;

    public DriveToLoadingStationCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision,
                                        Supplier<Side> loadingStationSide) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.loadingStationSide = loadingStationSide;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        shouldRegeneratePaths = true;
    }

    @Override
    public void execute() {
        if (!vision.doesIntakeHaveTarget() || !shouldRegeneratePaths){
            return;
        }

        NWTranslation2d predictedPosition = vision.getPredictedPostition();
        // We don't want to generate a path at is way too large due to bad vision data, so truncate it to a maximum distance
        if (predictedPosition.length > 240.0) {
            predictedPosition = (NWTranslation2d) predictedPosition.normal().times(240.0);
        }

        if (predictedPosition.getX() < 40.0) {
            shouldRegeneratePaths = false;
        }

        NWTranslation2d goal = new NWTranslation2d(20.0, 0.0);
        switch (loadingStationSide.get()) {
            case LEFT:
                goal = new NWTranslation2d(20.0, 22.0);
                break;
            case RIGHT:
                goal = new NWTranslation2d(20.0, -22.0);
                break;
        }

        Path path = new SimplePathBuilder(predictedPosition, NWRotation2d.ZERO)
                .lineTo(goal)
                .build();

        double startingVelocity = drivetrain.getVelocity().length;
        Path.State lastState = drivetrain.getFollower().getLastState();
        if (lastState != null) {
            startingVelocity = lastState.velocityMetersPerSecond;
        }

        Trajectory trajectory = new Trajectory(path, DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, 12.0, startingVelocity, 0.0);
        drivetrain.resetPose(new NWPose2d(predictedPosition, drivetrain.getPose().rotation));
        drivetrain.getFollower().follow2(trajectory);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.getFollower().cancel();
        drivetrain.drive(NWTranslation2d.ZERO, 0.0, false);
    }
}