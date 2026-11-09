package assignment;
 
import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.LinkedList;
import java.util.List;

public class Visualization {
    BoundedVolumeHierarchy boundingVolumeHierarchy;

    private static final int STEP = 5;
    private static final int HALF_SIZE = 10;
    private static final int CENTER = 200;
    private static final int FRAME_SIZE = 400;
    private int squareX;
    private int squareY;
 
    /**
     * Initialize state of square to be at (0, 0)
     */
    public Visualization () {
        squareX = 0;
        squareY = 0;
    }

    /**
     * @param boundingVolumeHierarchy        the BVH to set the boundedVolumeHierarchy to
     */
    public void setBoundingVolumeHierarchy(BoundedVolumeHierarchy boundingVolumeHierarchy) {
        this.boundingVolumeHierarchy = boundingVolumeHierarchy;
    }

    /**
     * Moves the square within the environment if it's able to move in the given direction
     *
     * @param keyPressed        the character of the keyPressed
     * @return              if the environment should be redrawn or not
     */
    public boolean moveSquare (char keyPressed) {
        int dx = 0;
        int dy = 0;
        switch (Character.toLowerCase(keyPressed)) {
            case 'w': dy = -STEP; break;
            case 's': dy = STEP; break;
            case 'a': dx = -STEP; break;
            case 'd': dx = STEP; break;
            default: return false;
        }
 
        int newX = squareX + dx;
        int newY = squareY + dy;
        if (squareHitsTriangle(newX, newY)) {
            return false;
        }
        squareX = newX;
        squareY = newY;
        return true;
    }

    /**
     * Calculates all the pixels visible from the square in the center
     *
     * @return              All points that should be drawn other than the square
     */
    public LinkedList<Point> drawGUI() {
        LinkedList<Point> pixels = new LinkedList<>();
        if (boundingVolumeHierarchy == null) {
            return pixels;
        }
 
        if (boundingVolumeHierarchy.findOverlappingBoundingBox(
                squareX - CENTER, squareY - CENTER, squareX + CENTER, squareY + CENTER).isEmpty()) {
            return pixels;
        }
 
        Point2D.Double origin = new Point2D.Double(squareX, squareY);
        for (int gx = 0; gx < FRAME_SIZE; gx++) {
            for (int gy = 0; gy < FRAME_SIZE; gy++) {
                if (gx == CENTER && gy == CENTER) {
                    continue;
                }
                Vector2D direction = new Vector2D(gx - CENTER, gy - CENTER);
                Point2D.Double hit = boundingVolumeHierarchy.intersectRayPoint(origin, direction);
                if (hit == null) {
                    continue;
                }
                long hitX = Math.round(hit.x - squareX + CENTER);
                long hitY = Math.round(hit.y - squareY + CENTER);
                if (hitX == gx && hitY == gy) {
                    pixels.add(new Point(gx, gy));
                }
            }
        }
        return pixels;
    } 
    
    //checks if square would hit any triangle after moving
    private boolean squareHitsTriangle(int cx, int cy) {
        if (boundingVolumeHierarchy == null) {
            return false;
        }
        int x0 = cx - HALF_SIZE;
        int y0 = cy - HALF_SIZE;
        int x1 = cx + HALF_SIZE;
        int y1 = cy + HALF_SIZE;

        //gets shapes overlap w square
        List<Shape> candidates = boundingVolumeHierarchy.findOverlappingBoundingBox(x0, y0, x1, y1);
        for (Shape s : candidates) {
            //blocks mvmt if shape isnt triangle
            if (!(s instanceof Triangle)) {
                return true;
            }
            //checks if triangle touches square
            if (triangleTouchesRect((Triangle) s, x0, y0, x1, y1)) {
                return true;
            }
        }
        return false;
    }

    //checks if triangle touches rectangle
    private static boolean triangleTouchesRect(Triangle t, int x0, int y0, int x1, int y1) {
        Point[] tri = {t.a, t.b, t.c};
        int[][] rect = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};

        //checks if triangle point is inside rectangle
        for (Point p : tri) {
            if (p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1) {
                return true;
            }
        }

        //checks if rectangle corner is inside triangle
        for (int[] corner : rect) {
            if (t.containsPoint(new Point2D.Double(corner[0], corner[1]))) {
                return true;
            }
        }

        //checks if any triangle edge crosses rectangle edge
        for (int i = 0; i < 3; i++) {
            Point p = tri[i];
            Point q = tri[(i + 1) % 3];
            for (int j = 0; j < 4; j++) {
                int[] r = rect[j];
                int[] s = rect[(j + 1) % 4];
                if (segmentsIntersect(p.x, p.y, q.x, q.y, r[0], r[1], s[0], s[1])) {
                    return true;
                }
            }
        }
        return false;
    }

    //finds orientation of 3 points
    private static long orientation(long ax, long ay, long bx, long by, long cx, long cy) {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    }

    //checks if point between two endpoints
    private static boolean onSegment(long ax, long ay, long bx, long by, long px, long py) {
        return px >= Math.min(ax, bx) && px <= Math.max(ax, bx)
                && py >= Math.min(ay, by) && py <= Math.max(ay, by);
    }

    //checks if 2 segments intersect
    private static boolean segmentsIntersect(long ax, long ay, long bx, long by,
                                             long cx, long cy, long dx, long dy) {
        //gets orientation per point
        long o1 = Long.signum(orientation(ax, ay, bx, by, cx, cy));
        long o2 = Long.signum(orientation(ax, ay, bx, by, dx, dy));
        long o3 = Long.signum(orientation(cx, cy, dx, dy, ax, ay));
        long o4 = Long.signum(orientation(cx, cy, dx, dy, bx, by));

        //checks if segments cross
        if (o1 != o2 && o3 != o4) {
            return true;
        }

        //checks cases when points lie on segments
        if (o1 == 0 && onSegment(ax, ay, bx, by, cx, cy)) return true;
        if (o2 == 0 && onSegment(ax, ay, bx, by, dx, dy)) return true;
        if (o3 == 0 && onSegment(cx, cy, dx, dy, ax, ay)) return true;
        return o4 == 0 && onSegment(cx, cy, dx, dy, bx, by);
    }
}