package frc.robot;

import javax.swing.*;
import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.FuelStruct;


public class GoToBallLine {
    // 2026 FRC field dimensions (in meters)
    static final double FIELD_LENGTH = 16.46; // 54 feet
    static final double FIELD_WIDTH = 8.23;   // 27 feet
    static final int PIXELS_PER_METER = 50;   // Scale factor for display

    public static NetworkTableInstance inst;
    public static NetworkTable table;
    public static NetworkTable poseTable;
    public static StructArraySubscriber<FuelStruct> fuelSub;
    public static StructSubscriber<Pose2d> poseSub;
    public static Pose2d currentPose = new Pose2d(0, 0, new Rotation2d());
    
    public static void main(String[] args) {
        // Attempt to pre-load the ntcorejni native library from the project's build folder so
        // direct 'java frc.robot.GoToBall' runs (or IDE runs) can find the JNI without requiring
        // users to manually set LD_LIBRARY_PATH or -Djava.library.path.
        try {
            String cwd = System.getProperty("user.dir");
            java.io.File jniDir = new java.io.File(cwd, "build/jni/release");
            if (jniDir.exists() && jniDir.isDirectory()) {
                // Ensure java.library.path contains the jniDir so System.loadLibrary can find it
                try {
                    String libPath = System.getProperty("java.library.path");
                    String path = jniDir.getAbsolutePath();
                    if (!libPath.contains(path)) {
                        System.setProperty("java.library.path", libPath + java.io.File.pathSeparator + path);

                        // Try to update internal ClassLoader paths (usr_paths) used by System.loadLibrary
                        try {
                            java.lang.reflect.Field usrPathsField = ClassLoader.class.getDeclaredField("usr_paths");
                            usrPathsField.setAccessible(true);
                            String[] paths = (String[]) usrPathsField.get(null);
                            boolean found = false;
                            for (String p : paths) if (p.equals(path)) { found = true; break; }
                            if (!found) {
                                String[] newPaths = java.util.Arrays.copyOf(paths, paths.length + 1);
                                newPaths[paths.length] = path;
                                usrPathsField.set(null, newPaths);
                            }
                        } catch (NoSuchFieldException nsf) {
                            // Fallback: clear sys_paths so the VM will reinitialize from java.library.path
                            try {
                                java.lang.reflect.Field sysPaths = ClassLoader.class.getDeclaredField("sys_paths");
                                sysPaths.setAccessible(true);
                                sysPaths.set(null, null);
                            } catch (NoSuchFieldException nsf2) {
                                // ignore
                            }
                        }
                    }
                } catch (Throwable t) {
                    System.err.println("Failed to update java.library.path: " + t.getMessage());
                }

                // Possible names (Linux: libntcorejni.so, Windows: ntcorejni.dll)
                java.io.File so = new java.io.File(jniDir, "libntcorejni.so");
                java.io.File dll = new java.io.File(jniDir, "ntcorejni.dll");
                java.io.File dylib = new java.io.File(jniDir, "libntcorejni.dylib");
                java.io.File toLoad = null;
                if (so.exists()) toLoad = so;
                else if (dll.exists()) toLoad = dll;
                else if (dylib.exists()) toLoad = dylib;

                if (toLoad != null) {
                    try {
                        System.out.println("Attempting to load native library: " + toLoad.getAbsolutePath());
                        System.load(toLoad.getAbsolutePath());
                        System.out.println("Loaded native library: " + toLoad.getName());
                    } catch (UnsatisfiedLinkError ule) {
                        System.err.println("Failed to System.load native library: " + ule.getMessage());
                    }
                }
            }
        } catch (Throwable t) {
            // Non-fatal. proceed and the usual RuntimeLoader will attempt to find the library.
            System.err.println("Preload check for ntcorejni failed: " + t.getMessage());
        }
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("VisionData");
        poseTable = inst.getTable("AdvantageKit");
        inst.startClient4("GoToBallViewer");
        inst.setServer("localhost", 5810); // IMPORTANT: if you are running sim, serverName should be localhost. If not, it should be your team number(10.22.7.2)
        fuelSub = table.getStructArrayTopic("vision_data", FuelStruct.struct).subscribe(new FuelStruct[0]);
        poseSub = poseTable.getStructTopic("RealOutputs/Odometry/Robot", Pose2d.struct).subscribe(new Pose2d());
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("FRC 2026 Field Pose Picker");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE); // If window is closed, program is stopped.
            frame.setSize(
                (int)(FIELD_WIDTH * PIXELS_PER_METER) + 50,
                (int)(FIELD_LENGTH * PIXELS_PER_METER) + 50
            );
            frame.setLocationRelativeTo(null);
            
            BallPanel panel = new BallPanel();
            frame.add(panel);
            
            frame.setVisible(true);
        });
    }

}

class Ball {
    double xMeters, yMeters;  // Field coordinates in meters
    int radius = 10;  // Ball radius in pixels
    
    public Ball(double xMeters, double yMeters) {
        this.xMeters = xMeters;
        this.yMeters = yMeters;
    }
    
    // Convert meter coordinates to pixel coordinates for display
    int getPixelX() {
        return (int)(xMeters * GoToBallLine.PIXELS_PER_METER) + 25;
    }
    
    int getPixelY() {
        return (int)(yMeters * GoToBallLine.PIXELS_PER_METER) + 25;
    }
    
    public boolean contains(int px, int py) {
        int dx = getPixelX() - px;
        int dy = getPixelY() - py;
        return (dx * dx + dy * dy) <= (radius * radius);
    }
}

class NavGrid {
    double fieldWidth, fieldHeight;
    double nodeSize;
    boolean[][] grid;
    
    public NavGrid(String jsonPath) {
        fieldWidth = 16.54;  // Default from navgrid
        fieldHeight = 8.07;  // Default from navgrid
        nodeSize = 0.3;      // Default
        
        try {
            String content = new String(Files.readAllBytes(new File(jsonPath).toPath()));
            parseJson(content);
        } catch (Exception e) {
            System.err.println("Failed to load navgrid: " + e.getMessage());
            e.printStackTrace();
            grid = new boolean[0][0];
        }
    }
    
    private void parseJson(String json) {
        // Extract field_size (x and y coordinates)
        fieldWidth = extractJsonDouble(json, "\"x\":");
        fieldHeight = extractJsonDouble(json, "\"y\":");
        
        // Extract nodeSizeMeters
        nodeSize = extractJsonDouble(json, "\"nodeSizeMeters\":");
        
        // Extract grid - the array starts with "grid":[[
        int gridIndex = json.indexOf("\"grid\":[[");
        if (gridIndex >= 0) {
            // Start parsing from [[ 
            int startIdx = gridIndex + 7; // Position of first [
            ArrayList<boolean[]> rows = new ArrayList<>();
            
            int i = startIdx;
            while (i < json.length()) {
                if (json.charAt(i) == '[') {
                    // Found start of a row
                    int end = json.indexOf(']', i);
                    if (end > i) {
                        String rowContent = json.substring(i + 1, end);
                        String[] values = rowContent.split(",");
                        boolean[] row = new boolean[values.length];
                        
                        for (int j = 0; j < values.length; j++) {
                            row[j] = values[j].trim().equals("true");
                        }
                        rows.add(row);
                        i = end + 1;
                    } else {
                        break;
                    }
                } else if (json.charAt(i) == ']' && i + 1 < json.length() && json.charAt(i + 1) == '}') {
                    // End of grid
                    break;
                } else {
                    i++;
                }
            }
            
            if (rows.size() > 0) {
                grid = new boolean[rows.size()][];
                for (int j = 0; j < rows.size(); j++) {
                    grid[j] = rows.get(j);
                }
                
                // Count walkable vs blocked
                System.out.println("NavGrid loaded: " + grid.length + " rows x " + grid[0].length + " cols");
            }
        }
    }
    
    private double extractJsonDouble(String json, String key) {
        int pos = json.indexOf(key);
        if (pos < 0) return 0;
        
        int start = pos + key.length();
        int end = json.length();
        
        // Find next comma or bracket
        for (int i = start; i < json.length(); i++) {
            char c = json.charAt(i);
            if (c == ',' || c == '}' || c == ']') {
                end = i;
                break;
            }
        }
        
        String numStr = json.substring(start, end).trim();
        try {
            return Double.parseDouble(numStr);
        } catch (NumberFormatException e) {
            return 0;
        }
    }
    
    // Check if a point (in meters) is walkable
    public boolean isWalkable(double xMeters, double yMeters) {
        if (grid.length == 0) return true;
        
        // Convert meters to grid coordinates
        int gridX = (int)(xMeters / nodeSize);
        int gridY = (int)(yMeters / nodeSize);
        
        // Bounds check
        if (gridX < 0 || gridX >= grid[0].length || gridY < 0 || gridY >= grid.length) {
            return false;
        }
        boolean result = grid[gridY][gridX];
        return result;
    }
}

class Obstacle {
    double xMeters, yMeters;  // Field coordinates in meters
    double radiusMeters;      // Radius in meters
    
    public Obstacle(double xMeters, double yMeters, double radiusMeters) {
        this.xMeters = xMeters;
        this.yMeters = yMeters;
        this.radiusMeters = radiusMeters;
    }
    
    // Check if a point (in meters) collides with this obstacle
    public boolean contains(double x, double y) {
        double dx = x - xMeters;
        double dy = y - yMeters;
        return (dx * dx + dy * dy) <= (radiusMeters * radiusMeters);
    }
}

class BallPanel extends JPanel {
    java.util.List<FuelStruct> vision_data = new java.util.ArrayList<>();
    StructArrayPublisher<Pose2d> waypointsPub = GoToBallLine.inst.getTable("VisionData")
    .getStructArrayTopic("DrawnWaypoints", Pose2d.struct).publish();
    double dragStartX = -1, dragStartY = -1;
    double dragEndX = -1, dragEndY = -1;
    boolean isDragging = false;
    private boolean isValidPosition(double xMeters, double yMeters) { // This is kinda broken and isn't good, will replace soon
        if (fieldImage == null) return true;  // Accept if no image
        // Convert field meters to image pixels
        int pixelX = (int)(xMeters * pixelsPerMeterX);
        int pixelY = (int)(yMeters * pixelsPerMeterY);
        // Bounds check
        if (pixelX < 0 || pixelX >= fieldImage.getWidth() || pixelY < 0 || pixelY >= fieldImage.getHeight()) {
            return false;
        }
        int rgb = fieldImage.getRGB(pixelX, pixelY);
        int r = (rgb >> 16) & 0xFF;
        int g = (rgb >> 8) & 0xFF;
        int b = rgb & 0xFF;
        // Reject RED pixels (obstacles) - R significantly higher than G and B
        boolean isRed = r > 100 && g < 110 && b < 110;
        // Reject BLUE pixels (obstacles) - B significantly higher than R and G
        // Stricter thresholds: lower B threshold, higher R/G thresholds
        boolean isBlue = b > 100 && r < 120 && g < 120;
        return !(isRed || isBlue);
    }

    private void detectFieldBoundaries() {
        if (fieldImage == null) return;
        int width = fieldImage.getWidth();
        int height = fieldImage.getHeight();
        // Find white rectangle boundaries - look for white pixels (RGB > 200) excluding dark obstacles
        int minPixelX = width, maxPixelX = 0;
        int minPixelY = height, maxPixelY = 0;
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                int rgb = fieldImage.getRGB(x, y);
                int r = (rgb >> 16) & 0xFF;
                int g = (rgb >> 8) & 0xFF;
                int b = rgb & 0xFF;
                // Check for white pixels that are not dark obstacles
                if (r > 200 && g > 200 && b > 200) {
                    if (x < minPixelX) minPixelX = x;
                    if (x > maxPixelX) maxPixelX = x;
                    if (y < minPixelY) minPixelY = y;
                    if (y > maxPixelY) maxPixelY = y;
                }
            }
        }
        // Convert pixel coordinates to field meters
        fieldBoundMinX = minPixelX / pixelsPerMeterX;
        fieldBoundMaxX = maxPixelX / pixelsPerMeterX;
        fieldBoundMinY = minPixelY / pixelsPerMeterY;
        fieldBoundMaxY = maxPixelY / pixelsPerMeterY;
        // Expand boundaries inward to stay INSIDE the white border lines
        // The white lines are the boundary, we want to fill the interior
        double marginX = 0.08;  // Margin to get inside the border
        double marginY = 0.08;
        fieldBoundMinX = Math.max(0, fieldBoundMinX + marginX);
        fieldBoundMaxX = Math.min(GoToBallLine.FIELD_WIDTH, fieldBoundMaxX - marginX);
        fieldBoundMinY = Math.max(0, fieldBoundMinY + marginY);
        fieldBoundMaxY = Math.min(GoToBallLine.FIELD_LENGTH, fieldBoundMaxY - marginY);
        System.out.println("White rectangle interior: X(" + String.format("%.2f", fieldBoundMinX) + 
            " to " + String.format("%.2f", fieldBoundMaxX) + "), Y(" + 
            String.format("%.2f", fieldBoundMinY) + " to " + String.format("%.2f", fieldBoundMaxY) + ")");
    }
    StructPublisher<Pose2d> publisher = GoToBallLine.inst.getTable("VisionData")
    .getStructTopic("TargetPoseClicked", Pose2d.struct).publish();

    ArrayList<Ball> balls;
    BufferedImage fieldImage;
    double fieldBoundMinX = 0.0, fieldBoundMaxX = 16.46;
    double fieldBoundMinY = 0.0, fieldBoundMaxY = 8.23;
    double pixelsPerMeterX, pixelsPerMeterY;
    FuelStruct hoveredBall = null;
    public BallPanel() {
        balls = new ArrayList<>();      
        
        // Load field image - ONLY dependency
        try {
            fieldImage = ImageIO.read(new File("src/main/java/frc/robot/f5h5pjh7whrmr0cwb1v9zgfp5r_result_0.png")); // Path of the field image
            int w = fieldImage.getWidth();
            int h = fieldImage.getHeight();
            BufferedImage rotated = new BufferedImage(h, w, fieldImage.getType());
            Graphics2D g2 = rotated.createGraphics();
            g2.translate(h, 0); // Recenters field image to prevent image from dissapearing from view
            g2.rotate(Math.PI / 2); // Since g2.rotate is in radians, pi/2 is 90 degrees instead of a rounded innacurate decimal
            g2.drawImage(fieldImage, 0, 0, null);
            g2.dispose();
            fieldImage = rotated;
            pixelsPerMeterX = fieldImage.getWidth() / GoToBallLine.FIELD_WIDTH;
            pixelsPerMeterY = fieldImage.getHeight() / GoToBallLine.FIELD_LENGTH;
            detectFieldBoundaries();
        } catch (Exception e) {
            System.err.println("Failed to load field image: " + e.getMessage());
            fieldImage = null;
            return;
        }
        
        javax.swing.Timer ntUpdateTimer = new javax.swing.Timer(100, evt -> {
            FuelStruct[] ballsRaw = GoToBallLine.fuelSub.get();
            Pose2d robotPose = GoToBallLine.currentPose;
            double robotX = robotPose.getX();
            double robotY = robotPose.getY();
            double robotTheta = robotPose.getRotation().getRadians();
            double cosTheta = Math.cos(robotTheta);
            double sinTheta = Math.sin(robotTheta);
            java.util.List<FuelStruct> fieldRelativeBalls = new java.util.ArrayList<>();
            for (FuelStruct ball : ballsRaw) {
                double fieldX = robotX + cosTheta * ball.x - sinTheta * ball.y;
                double fieldY = robotY + sinTheta * ball.x + cosTheta * ball.y;
                double rotatedX = fieldY;
                double rotatedY = fieldX;
                fieldRelativeBalls.add(new FuelStruct((float) rotatedX, (float) rotatedY));
            }
            vision_data = fieldRelativeBalls;
            repaint();
        });
        ntUpdateTimer.start();
        
        // Add mouse listener for clicks
        addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                int panelWidth = getWidth();
                int panelHeight = getHeight();
                double imagePixelsPerMeterX = panelWidth / GoToBallLine.FIELD_WIDTH;
                double imagePixelsPerMeterY = panelHeight / GoToBallLine.FIELD_LENGTH;
                dragStartX = e.getX() / imagePixelsPerMeterX;
                dragStartY = e.getY() / imagePixelsPerMeterY;
                dragEndX = dragStartX;
                dragEndY = dragStartY;
                isDragging = true;
                repaint();
            }
            @Override
            public void mouseReleased(MouseEvent e) {
                if (!isDragging) return;
                isDragging = false;
                int panelWidth = getWidth();
                int panelHeight = getHeight();
                double imagePixelsPerMeterX = panelWidth / GoToBallLine.FIELD_WIDTH;
                double imagePixelsPerMeterY = panelHeight / GoToBallLine.FIELD_LENGTH;
                dragEndX = e.getX() / imagePixelsPerMeterX;
                dragEndY = e.getY() / imagePixelsPerMeterY;

                // Generate waypoints spaced 0.5m apart along the line
                double spacing = 0.5;
                double dx = dragEndX - dragStartX;
                double dy = dragEndY - dragStartY;
                double lineLength = Math.sqrt(dx * dx + dy * dy);
                double angle = Math.atan2(dy, dx); // direction of line in display space
                // Un-rotate angle back to field coords (CW 90 degrees means subtract pi/2)
                double fieldAngle = angle - Math.PI / 2;

                int numPoints = Math.max(1, (int)(lineLength / spacing) + 1);
                Pose2d[] waypoints = new Pose2d[numPoints];
                for (int i = 0; i < numPoints; i++) {
                    double t = (numPoints == 1) ? 0 : (double) i / (numPoints - 1);
                    double displayX = dragStartX + t * dx;
                    double displayY = dragStartY + t * dy;
                    // Un-rotate CW back to original field coords
                    double fieldX = displayY;
                    double fieldY = displayX;
                    waypoints[i] = new Pose2d(fieldX, fieldY, new Rotation2d(fieldAngle));
                }
                waypointsPub.set(waypoints);
                System.out.println("Published " + numPoints + " waypoints");
                repaint();
            }
        });
        
        addMouseMotionListener(new MouseMotionAdapter() {
            @Override
            public void mouseDragged(MouseEvent e) {
                int panelWidth = getWidth();
                int panelHeight = getHeight();
                double imagePixelsPerMeterX = panelWidth / GoToBallLine.FIELD_WIDTH;
                double imagePixelsPerMeterY = panelHeight / GoToBallLine.FIELD_LENGTH;
                dragEndX = e.getX() / imagePixelsPerMeterX;
                dragEndY = e.getY() / imagePixelsPerMeterY;
                repaint();
            }
            @Override
            public void mouseMoved(MouseEvent e) {
                int panelWidth = getWidth();
                int panelHeight = getHeight();
                double imagePixelsPerMeterX = panelWidth / GoToBallLine.FIELD_WIDTH;
                double imagePixelsPerMeterY = panelHeight / GoToBallLine.FIELD_LENGTH;
                double mouseXMeters = e.getX() / imagePixelsPerMeterX;
                double mouseYMeters = e.getY() / imagePixelsPerMeterY;
                FuelStruct newHoveredBall = null;
                for (FuelStruct ball : vision_data) {
                    double dx = ball.x - mouseXMeters;
                    double dy = ball.y - mouseYMeters;
                    if (Math.sqrt(dx * dx + dy * dy) < 0.2) {
                        newHoveredBall = ball;
                        break;
                    }
                }
                if (newHoveredBall != hoveredBall) {
                    hoveredBall = newHoveredBall;
                    repaint();
                }
            }
        });
    }
    
    @Override
    protected void paintComponent(Graphics g) { // This draws all of the graphical stuff
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        if (fieldImage != null) {
            int panelWidth = getWidth();
            int panelHeight = getHeight();
            g2d.drawImage(fieldImage, 0, 0, panelWidth, panelHeight, this);
            double panelPixelsPerMeterX = panelWidth / GoToBallLine.FIELD_WIDTH;
            double panelPixelsPerMeterY = panelHeight / GoToBallLine.FIELD_LENGTH;
            for (FuelStruct ball : vision_data) {
                int pixelX = (int)(ball.x * panelPixelsPerMeterX);
                int pixelY = (int)(ball.y * panelPixelsPerMeterY);
                int pixelRadius = Math.max(3, (int)(0.1 * panelPixelsPerMeterX));
                g2d.setColor(Color.YELLOW);
                g2d.fillOval(pixelX - pixelRadius, pixelY - pixelRadius,
                            pixelRadius * 2, pixelRadius * 2);
                g2d.setColor(Color.BLACK);
                g2d.setStroke(new BasicStroke(2));
                g2d.drawOval(pixelX - pixelRadius, pixelY - pixelRadius,
                            pixelRadius * 2, pixelRadius * 2);
            }
            // Draw the drag line
            if (dragStartX >= 0 && dragEndX >= 0) {
                double panelPixelsPerMeterXLine = panelWidth / GoToBallLine.FIELD_WIDTH;
                double panelPixelsPerMeterYLine = panelHeight / GoToBallLine.FIELD_LENGTH;
                int x1 = (int)(dragStartX * panelPixelsPerMeterXLine);
                int y1 = (int)(dragStartY * panelPixelsPerMeterYLine);
                int x2 = (int)(dragEndX * panelPixelsPerMeterXLine);
                int y2 = (int)(dragEndY * panelPixelsPerMeterYLine);
                g2d.setColor(Color.CYAN);
                g2d.setStroke(new BasicStroke(3));
                g2d.drawLine(x1, y1, x2, y2);
                // Draw dots at each waypoint position along the line
                double dx = dragEndX - dragStartX;
                double dy = dragEndY - dragStartY;
                double lineLength = Math.sqrt(dx * dx + dy * dy);
                int numPoints = Math.max(1, (int)(lineLength / 0.5) + 1);
                for (int i = 0; i < numPoints; i++) {
                    double t = (numPoints == 1) ? 0 : (double) i / (numPoints - 1);
                    int wx = (int)((dragStartX + t * dx) * panelPixelsPerMeterXLine);
                    int wy = (int)((dragStartY + t * dy) * panelPixelsPerMeterYLine);
                    g2d.setColor(Color.CYAN);
                    g2d.fillOval(wx - 4, wy - 4, 8, 8);
                }
            }
        } else {
            g2d.setColor(Color.LIGHT_GRAY);
            g2d.fillRect(0, 0, getWidth(), getHeight());
        }
    }
}