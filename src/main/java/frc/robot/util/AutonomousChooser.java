package frc.robot.util;



import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import ExternalLib.JackInTheBotLib.math.Vector2;
import ExternalLib.NorthwoodLib.MathWrappers.NWPose2d;
import ExternalLib.NorthwoodLib.MathWrappers.NWRotation2d;
import ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;
import ExternalLib.JackInTheBotLib.control.Trajectory;
import ExternalLib.JackInTheBotLib.math.RigidTransform2;
import ExternalLib.JackInTheBotLib.math.Rotation2;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import com.pathplanner.lib.PathPlannerTrajectory;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    NWTranslation2d trajectoryVector; 
    double trajectoryVectorX;
    double trajectoryVectorY;






    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();
    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("testAuton", AutonomousMode.TEST_AUTON);
    }


    private Command getTestAutoCommand(RobotContainer container){
        SequentialCommandGroup command = new SequentialCommandGroup();
        resetRobotPose(command, container, trajectories.getTestAuton());
        return command;
    }



    public Command getCommand(RobotContainer container){
        switch(autonomousModeChooser.getSelected()){
            case TEST_AUTON: 
                return getTestAutoCommand(container);
        }
        return null;
    }
    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, PathPlannerTrajectory trajectory) {

       trajectoryVectorX = trajectory.getInitialState().poseMeters.getX();
       trajectoryVectorY = trajectory.getInitialState().poseMeters.getY();
       trajectoryVector.plus(trajectoryVectorX, trajectoryVectorY);

       



       




        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetGyroAngle(NWRotation2d.ZERO)));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new NWPose2d(trajectoryVector, NWRotation2d.ZERO)
                )));
    }





    private enum AutonomousMode {
        TEST_AUTON
     
    }



    
}
