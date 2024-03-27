package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class Pathfind {

    public static Command pathFind(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        PathConstraints constraints = new PathConstraints(
                2.5, 2.5,
                Units.degreesToRadians(480), Units.degreesToRadians(620));

        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints,
                0);

        return pathfindingCommand;
    }

    public static Command pathFindPoint(String point) {
        PathConstraints constraints = new PathConstraints(
                2.5, 2.5,
                Units.degreesToRadians(360), Units.degreesToRadians(480));
        Pose2d pose;
        boolean havePose = false;
        Command command;
        switch (point) {
            case "ShootA":
                pose = new Pose2d(4.545, 7.096, new Rotation2d(-147.381));
                havePose = true;
                break;
            case "A":
                pose = new Pose2d(2.681, 7.025, new Rotation2d(0));
                havePose = true;
                break; 
            case "B":
                pose = new Pose2d(4.367, 6.119, new Rotation2d(220.5));
                havePose = true;
                break;
            case "ShootB":
                pose = new Pose2d(4.509, 5.782, new Rotation2d(180));
                havePose = true;
                break;
            case "C":
                pose = new Pose2d(3.018, 4.256, new Rotation2d(0));
                havePose = true;
                break;
            default:
                havePose = false;
                pose = new Pose2d(0, 0, new Rotation2d(0));
                break;
        }
        if (havePose) {
            command = AutoBuilder.pathfindToPose(pose, constraints, 0);
        } else {
            command = new RunCommand(() -> {

            });
        }
        return command;
    }
}
