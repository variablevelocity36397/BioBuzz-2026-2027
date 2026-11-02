package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

// Plans a route to a target that goes around the outside of both HIVEs.
// Both HIVEs plus the lane between them are treated as one no-go box, so routes only use the
// outer side lanes, never the middle. The box is grown by MARGIN so the robot's body clears it,
// then we find the shortest path through its corners (straight line if nothing is in the way).
public class HiveAvoider {

    // TODO tune on field. HIVE footprints in inches, {minX, maxX, minY, maxY}
    private static final double[][] HIVES = {
            {44.5, 48.5, 49, 92},     // red
            {93, 97, 48, 92},         // blue
    };
    // one box covering both HIVEs and the middle lane between them
    private static final double[] NO_GO = {
            HIVES[0][0],
            HIVES[1][1],
            Math.min(HIVES[0][2], HIVES[1][2]),
            Math.max(HIVES[0][3], HIVES[1][3]),
    };
    private static final double MARGIN = 14;       // about half the robot width
    private static final double EDGE_SLACK = 0.5; // lets paths run along a grown box's own edge
    private static final double ESCAPE = 1;       // if we start inside a grown box, only avoid the real HIVE plus this
    private static final double CENTER_TOLERANCE = 2; // close enough to the middle lane's center to skip centering

    // Returns the waypoints after start, ending with target. null if no route was found.
    // Each waypoint's heading is the heading to have on arriving there, so driving leg to leg
    // with linear heading turns gradually over the whole route.
    public List<Pose> route(Pose start, Pose target) {
        List<Pose> path = new ArrayList<>();
        Pose from = start;   // where the outside route (and the turning) begins

        if (inside(start, NO_GO, 0)) {
            // stuck in the middle lane: center up, drive straight out the end nearest the target,
            // then route around the outside from there. No turning in the lane so the corners don't hit a HIVE.
            double centerX = (HIVES[0][1] + HIVES[1][0]) / 2;
            double topExit = NO_GO[3] + MARGIN, bottomExit = NO_GO[2] - MARGIN;
            double exitY = Math.abs(target.y() - topExit) < Math.abs(target.y() - bottomExit) ? topExit : bottomExit;

            if (Math.abs(start.x() - centerX) > CENTER_TOLERANCE) {
                path.add(new Pose(centerX, start.y(), start.heading()));
            }
            from = new Pose(centerX, exitY, start.heading());
            path.add(from);
        }

        List<Pose> rest = shortestRoute(from, target);
        if (rest == null) {
            return null;
        }

        // spread the turn evenly over the distance driven after the lane
        double total = 0;
        Pose prev = from;
        for (Pose p : rest) {
            total += prev.distance(p);
            prev = p;
        }
        double delta = Angle.normalizeSigned(target.heading() - start.heading());   // shortest way round
        double driven = 0;
        prev = from;
        for (int i = 0; i < rest.size(); i++) {
            Pose p = rest.get(i);
            driven += prev.distance(p);
            prev = p;
            double heading = i == rest.size() - 1 || total == 0
                    ? target.heading()
                    : start.heading() + delta * driven / total;
            path.add(new Pose(p.x(), p.y(), heading));
        }
        return path;
    }

    private List<Pose> shortestRoute(Pose start, Pose target) {
        // nodes: 0 = start, 1 = target, then the 4 corners of the grown no-go box (outer side lanes only)
        List<Pose> nodes = new ArrayList<>();
        nodes.add(start);
        nodes.add(target);
        double minX = NO_GO[0] - MARGIN, maxX = NO_GO[1] + MARGIN;
        double minY = NO_GO[2] - MARGIN, maxY = NO_GO[3] + MARGIN;
        nodes.add(new Pose(minX, minY, target.heading()));
        nodes.add(new Pose(minX, maxY, target.heading()));
        nodes.add(new Pose(maxX, minY, target.heading()));
        nodes.add(new Pose(maxX, maxY, target.heading()));

        // dijkstra from start to target, with only 6 nodes the simple O(n^2) version is plenty
        int n = nodes.size();
        double[] dist = new double[n];
        int[] prev = new int[n];
        boolean[] visited = new boolean[n];
        for (int i = 0; i < n; i++) {
            dist[i] = Double.POSITIVE_INFINITY;
            prev[i] = -1;
        }
        dist[0] = 0;

        for (int step = 0; step < n; step++) {
            int u = -1;
            for (int i = 0; i < n; i++) {
                if (!visited[i] && (u == -1 || dist[i] < dist[u])) {
                    u = i;
                }
            }
            if (u == -1 || dist[u] == Double.POSITIVE_INFINITY || u == 1) {
                break;
            }
            visited[u] = true;

            for (int v = 0; v < n; v++) {
                if (visited[v] || !clear(nodes.get(u), nodes.get(v), u == 0)) {
                    continue;
                }
                double d = dist[u] + nodes.get(u).distance(nodes.get(v));
                if (d < dist[v]) {
                    dist[v] = d;
                    prev[v] = u;
                }
            }
        }

        if (prev[1] == -1) {
            return null;
        }
        List<Pose> path = new ArrayList<>();
        for (int i = 1; i != 0; i = prev[i]) {
            path.add(nodes.get(i));
        }
        Collections.reverse(path);
        return path;
    }

    // true if the segment a -> b doesn't go through the grown no-go box
    private boolean clear(Pose a, Pose b, boolean fromStart) {
        double grow = MARGIN - EDGE_SLACK;
        // parked right next to the HIVEs on the outside: only avoid the real thing so we can still leave
        if (fromStart && inside(a, NO_GO, grow)) {
            grow = ESCAPE;
        }
        return !hitsGrown(a, b, NO_GO, grow);
    }

    private boolean hitsGrown(Pose a, Pose b, double[] box, double grow) {
        return segmentHitsBox(a, b, box[0] - grow, box[1] + grow, box[2] - grow, box[3] + grow);
    }

    private boolean inside(Pose p, double[] hive, double grow) {
        return p.x() > hive[0] - grow && p.x() < hive[1] + grow
                && p.y() > hive[2] - grow && p.y() < hive[3] + grow;
    }

    // Liang-Barsky: clip the segment to the box, if anything is left it hits
    private boolean segmentHitsBox(Pose a, Pose b, double minX, double maxX, double minY, double maxY) {
        double dx = b.x() - a.x();
        double dy = b.y() - a.y();
        double[] p = {-dx, dx, -dy, dy};
        double[] q = {a.x() - minX, maxX - a.x(), a.y() - minY, maxY - a.y()};
        double tEnter = 0, tExit = 1;
        for (int i = 0; i < 4; i++) {
            if (p[i] == 0) {
                if (q[i] < 0) {
                    return false;   // parallel to this edge and outside it
                }
            } else {
                double t = q[i] / p[i];
                if (p[i] < 0) {
                    tEnter = Math.max(tEnter, t);
                } else {
                    tExit = Math.min(tExit, t);
                }
            }
        }
        return tEnter < tExit;
    }
}
