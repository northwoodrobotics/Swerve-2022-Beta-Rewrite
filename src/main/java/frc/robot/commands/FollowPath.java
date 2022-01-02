package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import ExternalLib.JackInTheBotLib.control.Trajectory;
public class FollowPath extends CommandBase{
    private final DrivetrainSubsystem drivetrain;
    private final Trajectory trajectory;
    public FollowPath(DrivetrainSubsystem drivetrain, Trajectory trajectory) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;

       
    }

    @Override
    public void initialize() {
        drivetrain.getFollower().follow(trajectory);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.getFollower().cancel();
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getFollower().getCurrentTrajectory().isEmpty();
    }
}
    

