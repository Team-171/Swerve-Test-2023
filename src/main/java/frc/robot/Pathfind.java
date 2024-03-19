package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class Pathfind {

    public static Command pathFind(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        PathConstraints constraints = new PathConstraints(
                2.5, 2.5,
                Units.degreesToRadians(480), Units.degreesToRadians(620));

        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints,
                0
        );

        return pathfindingCommand;
    }
}
