package frc.robot.util;

import ExternalLib.JackInTheBotLib.control.*;
import ExternalLib.JackInTheBotLib.io.PathReader;
import ExternalLib.JackInTheBotLib.math.Rotation2;
import ExternalLib.JackInTheBotLib.math.Vector2;


import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class AutonomousTrajectories {
    private static final String TestAuton = "testAuton";

    private PathPlannerTrajectory testTrajectory = PathPlanner.loadPath(TestAuton, 8, 5);


    public PathPlannerTrajectory getTestAuton(){
        return testTrajectory;
    }















    
}
