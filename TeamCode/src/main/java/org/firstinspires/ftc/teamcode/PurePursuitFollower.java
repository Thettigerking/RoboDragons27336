package org.firstinspires.ftc.teamcode;

import java.util.List;

public class PurePursuitFollower {

    public static class Pose {
        public double x, y, heading;
        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public static class Waypoint {
        public double x, y, heading;
        public Waypoint(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public interface DriveInterface {
        void setDriveVelocity(double vx, double vy, double omega);
        default void stop() { setDriveVelocity(0, 0, 0); }
    }

    private List<Waypoint> path;
    private double lookahead;
    private double maxVel;
    private double finishTolerance = 2.0;     // inches
    private double headingKp = 2.0;

    public PurePursuitFollower(List<Waypoint> path, double lookahead, double maxVel) {
        this.path = path;
        this.lookahead = lookahead;
        this.maxVel = maxVel;
    }

    private double dist(double ax, double ay, double bx, double by) {
        double dx = ax - bx;
        double dy = ay - by;
        return Math.sqrt(dx*dx + dy*dy);
    }

    private Waypoint findLookahead(Pose robot) {
        Waypoint best = path.get(path.size() - 1);
        double bestDist = Double.MAX_VALUE;

        for (Waypoint w : path) {
            double d = Math.abs(dist(robot.x, robot.y, w.x, w.y) - lookahead);
            if (d < bestDist) {
                bestDist = d;
                best = w;
            }
        }
        return best;
    }

    public boolean followStep(Pose robot, double desiredVel, DriveInterface drive) {

        Waypoint goal = path.get(path.size()-1);
        double distToGoal = dist(robot.x, robot.y, goal.x, goal.y);

        if (distToGoal < finishTolerance) {
            drive.stop();
            return true;
        }

        Waypoint look = findLookahead(robot);

        double dx = look.x - robot.x;
        double dy = look.y - robot.y;

        double sin = Math.sin(-robot.heading);
        double cos = Math.cos(-robot.heading);

        double rx = cos*dx - sin*dy;
        double ry = sin*dx + cos*dy;

        double v = Math.min(desiredVel, maxVel);

        double curvature = 0;
        if (Math.abs(rx) > 1e-6)
            curvature = 2 * ry / (lookahead * lookahead);

        double omega = curvature * v;

        if (distToGoal < 12) {
            double headingError = normalize(goal.heading - robot.heading);
            omega += headingKp * headingError;
        }

        drive.setDriveVelocity(v, 0, omega);
        return false;
    }

    private double normalize(double a) {
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
}
