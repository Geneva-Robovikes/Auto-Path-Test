package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RobotContainer {
    private Drivetrain drive = new Drivetrain();

    List<Pose2d> GalacticSearchARed = Arrays.asList(new Pose2d(), 
        new Pose2d(2.286, 2.286, new Rotation2d(Math.toRadians(45))), 
        new Pose2d(3.810, 1.524, new Rotation2d(Math.toRadians(-90))),
        new Pose2d(4.572, 3.810, new Rotation2d(Math.toRadians(0))),
        new Pose2d(8.382, 3.810, new Rotation2d(Math.toRadians(0)))
    );

    List<Pose2d> GalacticSearchABlue = Arrays.asList(new Pose2d(),
        new Pose2d(4.572, .762, new Rotation2d(Math.toRadians(-90))),
        new Pose2d(5.334, 3.048, new Rotation2d(Math.toRadians(0))),
        new Pose2d(6.858, 2.286, new Rotation2d(Math.toRadians(0))),
        new Pose2d(8.382, 2.286, new Rotation2d(Math.toRadians(0)))
    );

    List<Pose2d> GalacticSearchBRed = Arrays.asList(new Pose2d(),
        new Pose2d(1.524, 0, new Rotation2d(Math.toRadians(90))),
        new Pose2d(3.048, -1.524, new Rotation2d(Math.toRadians(0))),
        new Pose2d(4.572, 1.524, new Rotation2d(Math.toRadians(0))),
        new Pose2d(7.62, 0, new Rotation2d(0))
    );

    List<Pose2d> GalacticSearchBBlue = Arrays.asList(new Pose2d(),
        new Pose2d(3.81, 0, new Rotation2d(Math.toRadians(-90))),
        new Pose2d(5.334, 1.524, new Rotation2d(Math.toRadians(0))),
        new Pose2d(6.858, -1.524, new Rotation2d(Math.toRadians(0))),
        new Pose2d(7.62, 0, new Rotation2d(Math.toRadians(0)))
    );

    public Command getCommand() {
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(1), Units.feetToMeters(1));

        config.setKinematics(drive.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(GalacticSearchBBlue, config);

        RamseteCommand command = new RamseteCommand(
            trajectory, 
            drive::getPose, 
            new RamseteController(2.0, 0.7),
            drive.getFeedForward(), 
            drive.getKinematics(),
            drive::getSpeeds, 
            drive.getLeftPIDController(), 
            drive.getRightPIDController(), 
            drive::setOutput,
            drive
        );

        return command;
    }

    public Drivetrain getDrive() {
        return drive;
    }
}
