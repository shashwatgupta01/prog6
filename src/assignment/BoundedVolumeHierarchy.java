package assignment;
 
import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

//bounding volume hierarchy for shapes, each leaf stores a shape, each internal node has smallest axis-aligned bounding box w shapes in its subtrees
public class BoundedVolumeHierarchy implements BVH {
    //tolerance used for floating-point coord comparison
    private static final double EPS = 1E-6;
 
    //a bvh node, leaf when shape is non-null; otherwise internal node w 2 children
    private static class Node {
        int minX, minY, maxX, maxY;
        Node left, right;
        Shape shape;
        int height;
 
        boolean isLeaf() {
            return shape != null;
        }
    }
 
    //stores closest shape hit
    private static class RayHit {
        Shape shape = null;
        Point2D.Double point = null;
        double distance = Double.POSITIVE_INFINITY;
    }
 
    //root of bvh, null root is empty hierarchy
    private Node root = null;
    private SplitMethod splitMethod = SPLIT_DEFAULT;
    private final Random random = new Random();
 
    //choose splitting strategy used by next buildBVH call, null argument resets strategy to interface default
    @Override
    public void setSplitMethod(SplitMethod splitMethod) {
        this.splitMethod = (splitMethod == null) ? SPLIT_DEFAULT : splitMethod;
    }
 
    //new hierarchy from supplied shapes
    @Override
    public void buildBVH(List<Shape> shapeList) {
        List<Shape> shapes = new ArrayList<>();
        if (shapeList != null) {
            for (Shape s : shapeList) {
                if (s != null) {
                    shapes.add(s);
                }
            }
        }
        root = shapes.isEmpty() ? null : build(shapes, 0);
    }
 
    //insert new shape by descending to subtree then updates bounding boxes/heights and tries to restore avl balance
    @Override
    public void insert(Shape shape) {
        if (shape == null) {
            return;
        }
        Node newLeaf = makeLeaf(shape);
        root = (root == null) ? newLeaf : insert(root, newLeaf);
    }

    //removes a matching shape, when leaf is removed, parent is replaced by remaining child, then ancestors are rebalanced
    @Override
    public void remove(Shape shape) {
        if (shape == null || root == null) {
            return;
        }
        removedFlag = false;
        root = remove(root, shape);
    }

    //give every shape w query point, bounding boxes checked first so entire subtrees can be skipped when cant hit
    @Override
    public Set<Shape> findCollision(Point2D.Double point) {
        Set<Shape> result = new HashSet<>();
        if (point != null) {
            collect(root, point, result);
        }
        return result;
    }

    //give nearest shape intersected by ray
    @Override
    public Shape intersectRay(Point2D.Double origin, Vector2D direction) {
        return nearestHit(origin, direction).shape;
    }

    //give intersection point for nearest ray hit
    public Point2D.Double intersectRayPoint(Point2D.Double origin, Vector2D direction) {
        return nearestHit(origin, direction).point;
    }
    //returns leaf shapes whose node bounding boxes overlap supplied axis-aligned query rectangle
    public List<Shape> findOverlappingBoundingBox(int minX, int minY, int maxX, int maxY) {
        List<Shape> result = new ArrayList<>();
        overlap(root, minX, minY, maxX, maxY, result);
        return result;
    }
 
    //returns root height and -1 for empty
    public int getHeight() {
        return root == null ? -1 : root.height;
    }
 
    //checks avl height condition at every internal node
    public boolean isBalanced() {
        return isBalanced(root);
    }
 
    //returns tree in preorder w a tab per level of depth
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        print(root, 0, sb);
        return sb.toString();
    }
 
    //makes leaf and copies its shapes bounds
    private static Node makeLeaf(Shape s) {
        Node n = new Node();
        n.shape = s;
        Point lo = s.getMinSurroundingPoint();
        Point hi = s.getMaxSurroundingPoint();
        n.minX = lo.x;
        n.minY = lo.y;
        n.maxX = hi.x;
        n.maxY = hi.y;
        n.height = 0;
        return n;
    }
 
    //make internal node whose children are supplied subtrees
    private static Node makeInternal(Node l, Node r) {
        Node n = new Node();
        n.left = l;
        n.right = r;
        refresh(n);
        return n;
    }
 
    //recomputes internal nodes height and bounding box from children
    private static void refresh(Node n) {
        n.height = 1 + Math.max(n.left.height, n.right.height);
        n.minX = Math.min(n.left.minX, n.right.minX);
        n.minY = Math.min(n.left.minY, n.right.minY);
        n.maxX = Math.max(n.left.maxX, n.right.maxX);
        n.maxY = Math.max(n.left.maxY, n.right.maxY);
    }
 
    //finds area of axis-aligned bounding box
    private static long area(int minX, int minY, int maxX, int maxY) {
        return (long) (maxX - minX) * (long) (maxY - minY);
    }
 
    private static long area(Node n) {
        return area(n.minX, n.minY, n.maxX, n.maxY);
    }
 
    //finds area of smallest box w both nodes
    private static long unionArea(Node a, Node b) {
        return area(Math.min(a.minX, b.minX), Math.min(a.minY, b.minY),
                Math.max(a.maxX, b.maxX), Math.max(a.maxY, b.maxY));
    }
 
    private static double coord(Point2D p, int axis) {
        return axis == 0 ? p.getX() : p.getY();
    }
 
    //recursively builds tree, split axis alternates by depth
    private Node build(List<Shape> shapes, int depth) {
        if (shapes.size() == 1) {
            return makeLeaf(shapes.get(0));
        }
 
        //alternate split axis at each level
        final int axis = depth % 2;
        List<Shape> sorted = new ArrayList<>(shapes);
        sorted.sort(Comparator.comparingDouble(s -> coord(s.getCenter(), axis)));
 
        //find coordinate of splitting line for this subtree
        double line = splitLine(sorted, axis);
 
        List<Shape> left = new ArrayList<>();
        List<Shape> right = new ArrayList<>();
        List<Shape> onLine = new ArrayList<>();
        for (Shape s : sorted) {
            double c = coord(s.getCenter(), axis);
            if (Shape.isClose(c, line)) {
                onLine.add(s);
            } else if (c < line) {
                left.add(s);
            } else {
                right.add(s);
            }
        }
 
        //distribute shapes centered on split line to keep groups even
        for (Shape s : onLine) {
            if (left.size() <= right.size()) {
                left.add(s);
            } else {
                right.add(s);
            }
        }
 
        //split must make progress, if a side is empty, isolate shape whose center is closest to splitting line
        if (left.isEmpty() || right.isEmpty()) {
            Shape nearest = sorted.get(0);
            double best = Double.POSITIVE_INFINITY;
            for (Shape s : sorted) {
                double d = Math.abs(coord(s.getCenter(), axis) - line);
                if (d < best) {
                    best = d;
                    nearest = s;
                }
            }
            List<Shape> others = new ArrayList<>(sorted);
            others.remove(nearest);
            List<Shape> isolated = new ArrayList<>();
            isolated.add(nearest);
            if (right.isEmpty()) {
                left = others;
                right = isolated;
            } else {
                left = isolated;
                right = others;
            }
        }
 
        return makeInternal(build(left, depth + 1), build(right, depth + 1));
    }
 
    //calculates split coordinate, midrange uses midpoint of subtree overall bounds; median uses middle shape-center val
    private double splitLine(List<Shape> sortedByCenter, int axis) {
        if (splitMethod == SplitMethod.SPLIT_MIDRANGE) {
            double min = Double.POSITIVE_INFINITY;
            double max = Double.NEGATIVE_INFINITY;
            for (Shape s : sortedByCenter) {
                min = Math.min(min, coord(s.getMinSurroundingPoint(), axis));
                max = Math.max(max, coord(s.getMaxSurroundingPoint(), axis));
            }
            return (min + max) / 2.0;
        }
        int n = sortedByCenter.size();
        if (n % 2 == 1) {
            return coord(sortedByCenter.get(n / 2).getCenter(), axis);
        }
        return (coord(sortedByCenter.get(n / 2 - 1).getCenter(), axis)
                + coord(sortedByCenter.get(n / 2).getCenter(), axis)) / 2.0;
    }
 
    //recursively inserts new leaf, reaching existing leaf makes new parent w old and new
    private Node insert(Node node, Node newLeaf) {
        if (node.isLeaf()) {
            return makeInternal(node, newLeaf);
        }
        if (chooseLeft(node, newLeaf)) {
            node.left = insert(node.left, newLeaf);
        } else {
            node.right = insert(node.right, newLeaf);
        }
        refresh(node);
        return rebalance(node);
    }
 
    //applies insertion priorities
    private boolean chooseLeft(Node node, Node newLeaf) {
        boolean leftIsLeaf = node.left.isLeaf();
        boolean rightIsLeaf = node.right.isLeaf();
        if (leftIsLeaf != rightIsLeaf) {
            return leftIsLeaf;
        }
        long growLeft = unionArea(node.left, newLeaf) - area(node.left);
        long growRight = unionArea(node.right, newLeaf) - area(node.right);
        if (growLeft != growRight) {
            return growLeft < growRight;
        }
        if (node.left.height != node.right.height) {
            return node.left.height < node.right.height;
        }
        return random.nextBoolean();
    }
 
    //communicates if remove operation found target
    private boolean removedFlag = false;
 
    //removes first matching leaf in this subtree, containment prunes branches that cant contain target Shape
    private Node remove(Node node, Shape target) {
        if (!covers(node, target)) {
            return node;
        }
        if (node.isLeaf()) {
            if (node.shape.equals(target)) {
                removedFlag = true;
                return null;
            }
            return node;
        }
 
        Node newLeft = remove(node.left, target);
        if (removedFlag) {
            if (newLeft == null) {
                return node.right;
            }
            node.left = newLeft;
        } else {
            Node newRight = remove(node.right, target);
            if (!removedFlag) {
                return node;
            }
            if (newRight == null) {
                return node.left;
            }
            node.right = newRight;
        }
        refresh(node);
        return rebalance(node);
    }
 
    //tests if nodes bounding box contains targets bounds
    private static boolean covers(Node n, Shape s) {
        Point lo = s.getMinSurroundingPoint();
        Point hi = s.getMaxSurroundingPoint();
        return n.minX <= lo.x && n.minY <= lo.y && n.maxX >= hi.x && n.maxY >= hi.y;
    }
 
    //tries tree rotation when child heights differ >1, rotation is also checked so local subtrees meet avl height condition
    private Node rebalance(Node n) {
        if (n.isLeaf()) {
            return n;
        }
        //positive means left subtree taller; negative means right
        int diff = n.left.height - n.right.height;
        if (diff >= -1 && diff <= 1) {
            return n;
        }
 
        Node heavy = diff > 0 ? n.left : n.right;
        Node light = diff > 0 ? n.right : n.left;
 
        Node promote = null;
        Node push = null;
        long bestArea = Long.MAX_VALUE;
        //try promoting either child of taller subtree, choose rotation that gives smaller combined area
        for (int i = 0; i < 2; i++) {
            Node g = (i == 0) ? heavy.left : heavy.right;
            Node o = (i == 0) ? heavy.right : heavy.left;
            int lowerHeight = 1 + Math.max(o.height, light.height);
            boolean valid = Math.abs(o.height - light.height) <= 1
                    && Math.abs(g.height - lowerHeight) <= 1;
            if (valid) {
                long a = unionArea(o, light);
                if (a < bestArea) {
                    bestArea = a;
                    promote = g;
                    push = o;
                }
            }
        }
        if (promote == null) {
            return n;
        }
 
        n.left = push;
        n.right = light;
        refresh(n);
        heavy.left = promote;
        heavy.right = n;
        refresh(heavy);
        return heavy;
    }
 
    //checks every internal node is avl balanced
    private static boolean isBalanced(Node n) {
        if (n == null || n.isLeaf()) {
            return true;
        }
        return Math.abs(n.left.height - n.right.height) <= 1
                && isBalanced(n.left) && isBalanced(n.right);
    }
 
    //tests if point lies in nodes bounding box
    private static boolean boxContains(Node n, Point2D.Double p) {
        return p.x >= n.minX - EPS && p.x <= n.maxX + EPS && p.y >= n.minY - EPS && p.y <= n.maxY + EPS;
    }
 
    //searches only subtrees whose bounding boxes contain point
    private void collect(Node n, Point2D.Double p, Set<Shape> out) {
        if (n == null || !boxContains(n, p)) {
            return;
        }
        if (n.isLeaf()) {
            if (n.shape.containsPoint(p)) {
                out.add(n.shape);
            }
            return;
        }
        collect(n.left, p, out);
        collect(n.right, p, out);
    }
 
    //collects shapes from nodes whose bounds overlap query rectangle
    private void overlap(Node n, int minX, int minY, int maxX, int maxY, List<Shape> out) {
        if (n == null || n.maxX < minX || n.minX > maxX || n.maxY < minY || n.minY > maxY) {
            return;
        }
        if (n.isLeaf()) {
            out.add(n.shape);
            return;
        }
        overlap(n.left, minX, minY, maxX, maxY, out);
        overlap(n.right, minX, minY, maxX, maxY, out);
    }
 
    //starts nearest-ray-hit search
    private RayHit nearestHit(Point2D.Double origin, Vector2D direction) {
        RayHit best = new RayHit();
        if (root == null || origin == null || direction == null) {
            return best;
        }
        Point intOrigin = new Point((int) Math.round(origin.x), (int) Math.round(origin.y)); //round origin to integer point for Shape.findIntersection()
        traverseRay(root, origin, intOrigin, direction, best);
        return best;
    }
 
    //traverses bounding boxes in increasing distance
    private void traverseRay(Node n, Point2D.Double origin, Point intOrigin, Vector2D dir, RayHit best) {
        double entry = boxEntryDistance(n, origin, dir);
        if (entry < 0 || entry > best.distance + EPS) {
            return;
        }
        if (n.isLeaf()) {
            Point2D.Double hit = n.shape.findIntersection(intOrigin, dir);
            if (hit != null) {
                double d = BVH.distanceBetweenPoints(origin, hit);
                if (d < best.distance) {
                    best.distance = d;
                    best.shape = n.shape;
                    best.point = hit;
                }
            }
            return;
        }
        double el = boxEntryDistance(n.left, origin, dir);
        double er = boxEntryDistance(n.right, origin, dir);
        if (el >= 0 && (er < 0 || el <= er)) {
            traverseRay(n.left, origin, intOrigin, dir, best);
            traverseRay(n.right, origin, intOrigin, dir, best);
        } else {
            traverseRay(n.right, origin, intOrigin, dir, best);
            traverseRay(n.left, origin, intOrigin, dir, best);
        }
    }
 
    //uses slab method to calculate rays entry distance into box, return -1 when miss
    private static double boxEntryDistance(Node n, Point2D.Double o, Vector2D d) {
        double tMin = 0;
        double tMax = Double.POSITIVE_INFINITY;
        for (int axis = 0; axis < 2; axis++) {
            double origin = axis == 0 ? o.x : o.y;
            double dir = axis == 0 ? d.getX() : d.getY();
            double lo = (axis == 0 ? n.minX : n.minY) - EPS;
            double hi = (axis == 0 ? n.maxX : n.maxY) + EPS;
            if (dir == 0) {
                if (origin < lo || origin > hi) {
                    return -1;
                }
            } else {
                double t1 = (lo - origin) / dir;
                double t2 = (hi - origin) / dir;
                if (t1 > t2) {
                    double tmp = t1;
                    t1 = t2;
                    t2 = tmp;
                }
                tMin = Math.max(tMin, t1);
                tMax = Math.min(tMax, t2);
                if (tMin > tMax) {
                    return -1;
                }
            }
        }
        return tMin * Math.hypot(d.getX(), d.getY());
    }
 
    //appends this subtrees preorder representation
    private void print(Node n, int depth, StringBuilder sb) {
        if (n == null) {
            return;
        }
        for (int i = 0; i < depth; i++) {
            sb.append('\t');
        }
        if (n.isLeaf()) {
            if (n.shape instanceof Triangle) {
                Triangle t = (Triangle) n.shape;
                sb.append('(').append(t.a.x).append(", ").append(t.a.y).append(") ");
                sb.append('(').append(t.b.x).append(", ").append(t.b.y).append(") ");
                sb.append('(').append(t.c.x).append(", ").append(t.c.y).append(")");
            } else {
                sb.append(n.shape.toString().trim());
            }
            sb.append('\n');
        } else {
            sb.append("[(").append(n.minX).append(", ").append(n.minY).append("), (")
                    .append(n.maxX).append(", ").append(n.maxY).append(")]\n");
            print(n.left, depth + 1, sb);
            print(n.right, depth + 1, sb);
        }
    }
}