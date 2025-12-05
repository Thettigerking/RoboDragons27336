package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueGoalCloseAuto", group = "Autonomous")
@Configurable // Panels
public class BlueGoalCloseAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33.5, 135.5, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(34.000, 135.500), new Pose(59.000, 97.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(315))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.000, 97.000), new Pose(58.000, 97.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                    .build();
//10
            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.000, 97.000), new Pose(21.000, 97.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.000, 97.000), new Pose(72.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 72.000), new Pose(42.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.000, 60.000), new Pose(21.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.000, 60.000), new Pose(72.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 72.000), new Pose(42.000, 35.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.000, 35.500), new Pose(21.000, 35.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.000, 35.500), new Pose(72.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 72.000), new Pose(105.000, 33.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        if (!follower.isBusy()) {

            switch (pathState) {

                case 0:
                    follower.followPath(paths.Path1);
                    pathState++;
                    break;

                case 1:
                    follower.followPath(paths.Path2);
                    pathState++;
                    break;

                case 2:
                    follower.followPath(paths.Path3);
                    pathState++;
                    break;

                case 3:
                    follower.followPath(paths.Path4);
                    pathState++;
                    break;

                case 4:
                    follower.followPath(paths.Path5);
                    pathState++;
                    break;

                case 5:
                    follower.followPath(paths.Path6);
                    pathState++;
                    break;

                case 6:
                    follower.followPath(paths.Path7);
                    pathState++;
                    break;

                case 7:
                    follower.followPath(paths.Path8);
                    pathState++;
                    break;
                case 8:
                    follower.followPath(paths.Path9);
                    pathState++;
                    break;
                case 9:
                    follower.followPath(paths.Path10);
                    pathState++;
                    break;
                case 10:
                    follower.followPath(paths.Path11);
                    pathState++;
                    break;
            }
        }

        return pathState;
    }

}