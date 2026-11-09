package assignment;
 
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
 
import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
 
import static org.junit.jupiter.api.Assertions.*;

/* 
 * Any comments and methods here are purely descriptions or suggestions.
 * This is your test file. Feel free to change this as much as you want.
 */

public class BVHTest {
    private static List<Shape> figure;
    private static final String FIGURE_STRING =
            "[(0, 0), (5, 5)]\n\t[(0, 0), (3, 5)]\n\t\t(0, 0) (1, 0) (1, 2)\n\t\t(1, 5) (2, 3) (3, 5)\n"
            + "\t[(2, 1), (5, 5)]\n\t\t(2, 2) (3, 1) (4, 2)\n\t\t(4, 5) (5, 3) (5, 5)\n";
    private BoundedVolumeHierarchy bvh;
 
    @BeforeAll
    static void setupAll() {
        figure = List.of(tri(0, 0, 1, 0, 1, 2), tri(1, 5, 2, 3, 3, 5), tri(2, 2, 3, 1, 4, 2), tri(4, 5, 5, 3, 5, 5));
    }
 
    @BeforeEach
    void setupEach() {
        bvh = new BoundedVolumeHierarchy();
    }
 
    private static Triangle tri(int ax, int ay, int bx, int by, int cx, int cy) {
        return new Triangle(new Point(ax, ay), new Point(bx, by), new Point(cx, cy));
    }
 
    private static Point2D.Double pt(double x, double y) {
        return new Point2D.Double(x, y);
    }
 
    private static List<Shape> random(int n, long seed) {
        Random r = new Random(seed);
        List<Shape> list = new ArrayList<>();
        while (list.size() < n) {
            int x = r.nextInt(300), y = r.nextInt(300);
            Triangle t = tri(x, y, x + 1 + r.nextInt(8), y + r.nextInt(8), x + r.nextInt(8), y + 1 + r.nextInt(8));
            if ((t.b.x - t.a.x) * (t.c.y - t.a.y) != (t.b.y - t.a.y) * (t.c.x - t.a.x)) {
                list.add(t);
            }
        }
        return list;
    }
 
    private int pos;
    private String[] lines;
 
    private boolean balanced(String tree) {
        lines = tree.split("\n");
        pos = 0;
        return tree.isEmpty() || height() >= 0;
    }
 
    private int height() {
        if (!lines[pos++].trim().startsWith("[")) {
            return 0;
        }
        int l = height(), r = height();
        return (l < 0 || r < 0 || Math.abs(l - r) > 1) ? -1 : 1 + Math.max(l, r);
    }
 
    private void assertHasAll(List<Shape> shapes) {
        for (Shape s : shapes) {
            assertTrue(bvh.findCollision(s.getCenter()).contains(s), "lost " + s);
        }
    }
 
    @Test
    void testBuildMatchesSpecExample() {
        bvh.buildBVH(figure);
        assertEquals(FIGURE_STRING, bvh.toString());
    }
 
    @Test
    void testBuildEmptyAndSingle() {
        bvh.buildBVH(new ArrayList<>());
        assertEquals("", bvh.toString());
        assertTrue(bvh.findCollision(pt(0, 0)).isEmpty());
        assertNull(bvh.intersectRay(pt(0, 0), new Vector2D(1, 0)));
 
        Triangle only = tri(0, 0, 4, 0, 0, 4);
        bvh.buildBVH(List.of(only));
        assertEquals("(0, 0) (4, 0) (0, 4)\n", bvh.toString());
        assertTrue(bvh.findCollision(pt(1, 1)).contains(only));
    }
 
    @Test
    void testBuildIsBalanced() {
        for (int n = 1; n <= 70; n++) {
            List<Shape> shapes = random(n, n);
            bvh.buildBVH(shapes);
            assertTrue(balanced(bvh.toString()), "unbalanced for n=" + n);
            assertHasAll(shapes);
        }
    }
 
    @Test
    void testSplitMethods() {
        List<Shape> same = new ArrayList<>();
        for (int i = 1; i <= 5; i++) {
            same.add(tri(-i, -i, i, -i, 0, 2 * i));
        }
        for (BVH.SplitMethod m : new BVH.SplitMethod[]{BVH.SplitMethod.SPLIT_MIDRANGE, BVH.SplitMethod.SPLIT_MEDIAN}) {
            bvh.setSplitMethod(m);
            List<Shape> shapes = random(60, 42);
            bvh.buildBVH(shapes);
            assertHasAll(shapes);
            bvh.buildBVH(same);
            assertHasAll(same);
        }
    }
 
    @Test
    void testInsert() {
        Triangle a = tri(0, 0, 2, 0, 0, 2), b = tri(10, 10, 12, 10, 10, 12);
        bvh.insert(a);
        assertEquals("(0, 0) (2, 0) (0, 2)\n", bvh.toString());
        bvh.insert(b);
        assertEquals("[(0, 0), (12, 12)]\n", bvh.toString().split("(?<=\n)")[0]);
 
        List<Shape> shapes = random(200, 7);
        for (Shape s : shapes) {
            bvh.insert(s);
            assertTrue(balanced(bvh.toString()), "unbalanced after inserting " + s);
        }
        assertHasAll(shapes);
        assertHasAll(List.of(a, b));
    }
 
    @Test
    void testRemoveFromSpecExample() {
        bvh.buildBVH(figure);
        bvh.remove(figure.get(1));
        assertEquals("[(0, 0), (5, 5)]\n\t(0, 0) (1, 0) (1, 2)\n\t[(2, 1), (5, 5)]\n"
                + "\t\t(2, 2) (3, 1) (4, 2)\n\t\t(4, 5) (5, 3) (5, 5)\n", bvh.toString());
        assertTrue(bvh.findCollision(pt(2, 4.5)).isEmpty());
    }
 
    @Test
    void testRemove() {
        List<Shape> remaining = random(120, 99);
        bvh.buildBVH(remaining);
        String before = bvh.toString();
        bvh.remove(tri(900, 900, 901, 900, 900, 901));
        assertEquals(before, bvh.toString());
 
        Random r = new Random(5);
        while (remaining.size() > 1) {
            bvh.remove(remaining.remove(r.nextInt(remaining.size())));
            assertTrue(balanced(bvh.toString()), "unbalanced after removal");
        }
        assertHasAll(remaining);
        bvh.remove(remaining.get(0));
        assertEquals("", bvh.toString());
        bvh.remove(remaining.get(0));
        assertEquals("", bvh.toString());
    }
 
    @Test
    void testFindCollision() {
        Triangle a = tri(0, 0, 10, 0, 0, 10), b = tri(2, 2, 12, 2, 2, 12), c = tri(50, 50, 60, 50, 50, 60);
        bvh.buildBVH(List.of(a, b, c));
        assertEquals(Set.of(a), bvh.findCollision(pt(1, 1)));
        assertEquals(Set.of(a, b), bvh.findCollision(pt(3, 3)));
        assertEquals(Set.of(c), bvh.findCollision(pt(51, 51)));
        assertTrue(bvh.findCollision(pt(30, 30)).isEmpty());
        assertTrue(bvh.findCollision(pt(9, 9)).isEmpty());
        assertTrue(bvh.findCollision(pt(0, 0)).contains(a));
    }
 
    @Test
    void testIntersectRayBasic() {
        Triangle near = tri(10, -5, 10, 5, 14, 0), far = tri(30, -5, 30, 5, 34, 0), off = tri(10, 40, 10, 50, 14, 45);
        bvh.buildBVH(List.of(far, near, off));
        assertEquals(near, bvh.intersectRay(pt(0, 0), new Vector2D(1, 0)));
        assertEquals(far, bvh.intersectRay(pt(20, 0), new Vector2D(1, 0)));
        assertEquals(near, bvh.intersectRay(pt(0, -10), new Vector2D(1, 1)));
        assertEquals(off, bvh.intersectRay(pt(0, 45), new Vector2D(1, 0)));
        assertNull(bvh.intersectRay(pt(0, 0), new Vector2D(-1, 0)));
        assertNull(bvh.intersectRay(pt(0, 0), new Vector2D(0, 1)));
        assertNull(bvh.intersectRay(pt(0, 100), new Vector2D(1, 0)));
    }
 
    @Test
    void testIntersectRayMatchesBruteForce() {
        Random r = new Random(2024);
        List<Shape> shapes = random(80, 300);
        bvh.buildBVH(shapes);
        for (int i = 0; i < 1000; i++) {
            Point o = new Point(r.nextInt(300), r.nextInt(300));
            int dx = r.nextInt(21) - 10, dy = r.nextInt(21) - 10;
            if (dx == 0 && dy == 0) {
                continue;
            }
            Vector2D d = new Vector2D(dx, dy);
            double best = Double.POSITIVE_INFINITY;
            for (Shape s : shapes) {
                Point2D.Double hit = s.findIntersection(o, d);
                best = hit == null ? best : Math.min(best, hit.distance(o));
            }
            Shape got = bvh.intersectRay(new Point2D.Double(o.x, o.y), d);
            if (best == Double.POSITIVE_INFINITY) {
                assertNull(got, "expected no hit");
            } else {
                assertNotNull(got, "expected a hit");
                assertEquals(best, got.findIntersection(o, d).distance(o), 1e-6, "not the nearest triangle");
            }
        }
    }
 
    private Visualization figure3() {
        bvh.buildBVH(List.of(tri(0, 100, 80, 20, 80, 100)));
        Visualization v = new Visualization();
        v.setBoundingVolumeHierarchy(bvh);
        return v;
    }
 
    @Test
    void testVisualizationMovement() {
        Visualization free = new Visualization();
        assertTrue(free.moveSquare('w'));
        assertTrue(free.moveSquare('A'));
        assertFalse(free.moveSquare('x'));
 
        Visualization v = figure3();
        int steps = 0;
        while (v.moveSquare('s')) {
            assertTrue(++steps < 100, "walked through the triangle");
        }
        assertTrue(steps > 5 && steps < 20, "blocked at unexpected place: " + steps);
        assertEquals(v.drawGUI(), v.drawGUI());
        assertTrue(v.moveSquare('w'));
    }
 
    @Test
    void testVisualizationDrawGUI() {
        assertTrue(new Visualization().drawGUI().isEmpty());
 
        Visualization v = figure3();
        Set<Point> px = new HashSet<>(v.drawGUI());
        assertTrue(px.contains(new Point(200, 300)) && px.contains(new Point(240, 260)) && px.contains(new Point(280, 220)));
        assertFalse(px.contains(new Point(280, 260)), "vertical leg is hidden");
        assertFalse(px.contains(new Point(240, 300)), "bottom leg is hidden");
        assertFalse(px.contains(new Point(200, 200)), "square is not part of the result");
 
        assertTrue(v.moveSquare('d'));
        Set<Point> shifted = new HashSet<>(v.drawGUI());
        assertTrue(shifted.contains(new Point(195, 300)) && shifted.contains(new Point(235, 260)));
        assertFalse(shifted.contains(new Point(240, 260)));
    }
}