package frc.robot;

import javax.swing.*;
import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.io.File;
import java.nio.file.Files;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class GoToBall {
    // 2026 FRC field dimensions (in meters)
    static final double FIELD_LENGTH = 16.46; // 54 feet
    static final double FIELD_WIDTH = 8.23;   // 27 feet
    static final int PIXELS_PER_METER = 50;   // Scale factor for display
    static final double robotX = 1.0;
    static final double robotY = 4.3;
    static final double robotRot = 36.0;
    
    
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
            // Non-fatal; we'll proceed and the usual RuntimeLoader will attempt to find the library.
            System.err.println("Preload check for ntcorejni failed: " + t.getMessage());
        }

        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("FRC 2026 Field Pose Picker");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setSize(
                (int)(FIELD_LENGTH * PIXELS_PER_METER) + 50,
                (int)(FIELD_WIDTH * PIXELS_PER_METER) + 50
            );
            frame.setLocationRelativeTo(null);
            
            BallPanel panel = new BallPanel();
            frame.add(panel);
            
            frame.setVisible(true);
        });
    }

}

class FuelStruct {
    public double xMeters, yMeters;
    public int radius;
    public FuelStruct(double xMeters, double yMeters, int radius) {
        this.xMeters = xMeters;
        this.yMeters = yMeters;
        this.radius = radius;
    }
    int getPixelX() {
        return (int)(xMeters * GoToBall.PIXELS_PER_METER) + 25;
    }
    int getPixelY() {
        return (int)(yMeters * GoToBall.PIXELS_PER_METER) + 25;
    }
    public boolean contains(int px, int py) {
        int dx = getPixelX() - px;
        int dy = getPixelY() - py;
        return (dx * dx + dy * dy) <= (radius * radius);
    }
}

class BallPanel extends JPanel {
    java.util.List<FuelStruct> vision_data;
    BufferedImage fieldImage;
    double fieldBoundMinX = 0, fieldBoundMaxX = 16.46;
    double fieldBoundMinY = 0, fieldBoundMaxY = 8.23;
    double pixelsPerMeterX, pixelsPerMeterY;
    FuelStruct hoveredBall = null;
    private DoubleArraySubscriber ballsXSub = null;
    private DoubleArraySubscriber ballsYSub = null;
    private javax.swing.Timer ntPoller = null;
    private javax.swing.Timer ntFallbackTimer = null;

    public BallPanel() {
        vision_data = new java.util.ArrayList<>();
        try {
            fieldImage = ImageIO.read(new File("src/main/java/frc/robot/f5h5pjh7whrmr0cwb1v9zgfp5r_result_0.png"));
            pixelsPerMeterX = fieldImage.getWidth() / GoToBall.FIELD_LENGTH;
            pixelsPerMeterY = fieldImage.getHeight() / GoToBall.FIELD_WIDTH;
            detectFieldBoundaries();
        } catch (Exception e) {
            System.err.println("Failed to load field image: " + e.getMessage());
            fieldImage = null;
            return;
        }
        boolean loadedFromNT = updateBallsFromNetwork();
        if (loadedFromNT) {
            System.out.println("Placed " + vision_data.size() + " balls from NetworkTables");
        } else {
            ntFallbackTimer = new javax.swing.Timer(5000, (ev) -> {
                boolean nowLoaded = updateBallsFromNetwork();
                if (nowLoaded) {
                    System.out.println("Placed " + vision_data.size() + " balls from NetworkTables (delayed)");
                }
                ((javax.swing.Timer) ev.getSource()).stop();
            });
            ntFallbackTimer.setRepeats(false);
            ntFallbackTimer.start();
        }
        addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                int panelWidth = getWidth();
                int panelHeight = getHeight();
                double imagePixelsPerMeterX = panelWidth / GoToBall.FIELD_LENGTH;
                double imagePixelsPerMeterY = panelHeight / GoToBall.FIELD_WIDTH;
                double clickXMeters = e.getX() / imagePixelsPerMeterX;
                double clickYMeters = e.getY() / imagePixelsPerMeterY;
                for (FuelStruct ball : vision_data) {
                    double dx = ball.xMeters - clickXMeters;
                    double dy = ball.yMeters - clickYMeters;
                    double distMeters = Math.sqrt(dx * dx + dy * dy);
                    if (distMeters < 0.2) {
                        System.out.printf("new Pose2d(%.2f, %.2f, new Rotation2d())%n",
                            ball.xMeters, ball.yMeters);
                        break;
                    }
                }
            }
            @Override
            public void mouseExited(MouseEvent e) {
                if (hoveredBall != null) {
                    hoveredBall = null;
                    repaint();
                }
            }
        });
        addMouseMotionListener(new MouseMotionAdapter() {
            @Override
            public void mouseMoved(MouseEvent e) {
                int panelWidth = getWidth();
                int panelHeight = getHeight();
                double imagePixelsPerMeterX = panelWidth / GoToBall.FIELD_LENGTH;
                double imagePixelsPerMeterY = panelHeight / GoToBall.FIELD_WIDTH;
                double mouseXMeters = e.getX() / imagePixelsPerMeterX;
                double mouseYMeters = e.getY() / imagePixelsPerMeterY;
                FuelStruct newHoveredBall = null;
                for (FuelStruct ball : vision_data) {
                    double dx = ball.xMeters - mouseXMeters;
                    double dy = ball.yMeters - mouseYMeters;
                    double distMeters = Math.sqrt(dx * dx + dy * dy);
                    if (distMeters < 0.2) {
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
    
    private boolean isValidPosition(double xMeters, double yMeters) {
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
        fieldBoundMaxX = Math.min(GoToBall.FIELD_LENGTH, fieldBoundMaxX - marginX);
        fieldBoundMinY = Math.max(0, fieldBoundMinY + marginY);
        fieldBoundMaxY = Math.min(GoToBall.FIELD_WIDTH, fieldBoundMaxY - marginY);
        
        System.out.println("White rectangle interior: X(" + String.format("%.2f", fieldBoundMinX) + 
            " to " + String.format("%.2f", fieldBoundMaxX) + "), Y(" + 
            String.format("%.2f", fieldBoundMinY) + " to " + String.format("%.2f", fieldBoundMaxY) + ")");
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        if (fieldImage != null) {
            int panelWidth = getWidth();
            int panelHeight = getHeight();
            g2d.drawImage(fieldImage, 0, 0, panelWidth, panelHeight, this);
            double panelPixelsPerMeterX = panelWidth / GoToBall.FIELD_LENGTH;
            double panelPixelsPerMeterY = panelHeight / GoToBall.FIELD_WIDTH;
            for (FuelStruct ball : vision_data) {
                int pixelX = (int)(ball.xMeters * panelPixelsPerMeterX);
                int pixelY = (int)(ball.yMeters * panelPixelsPerMeterY);
                int pixelRadius = Math.max(3, (int)(0.1 * panelPixelsPerMeterX));
                if (ball == hoveredBall) {
                    pixelRadius = (int)(pixelRadius * 2);
                }
                g2d.setColor(Color.YELLOW);
                g2d.fillOval(pixelX - pixelRadius, pixelY - pixelRadius,
                             pixelRadius * 2, pixelRadius * 2);
                g2d.setColor(Color.BLACK);
                g2d.setStroke(new BasicStroke(2));
                g2d.drawOval(pixelX - pixelRadius, pixelY - pixelRadius,
                            pixelRadius * 2, pixelRadius * 2);
            }
        } else {
            g2d.setColor(Color.LIGHT_GRAY);
            g2d.fillRect(0, 0, getWidth(), getHeight());
        }
    }

    // Initialize NetworkTables subscribers and start a periodic poller
    // NOTE: publishers are now expected to be named "FuelPointsX" and "FuelPointsY".
    // The incoming arrays are provided in inches (robot-relative); we convert inches -> meters
    // (1 in = 0.0254 m) before transforming into field coordinates.
    private void initNetworkTablesSubscribers() {
        try {
            var inst = NetworkTableInstance.getDefault();
            ballsXSub = inst.getDoubleArrayTopic("FuelPointsX").subscribe(new double[0]);
            ballsYSub = inst.getDoubleArrayTopic("FuelPointsY").subscribe(new double[0]);

            ntPoller = new javax.swing.Timer(500, (ev) -> {
                boolean changed = updateBallsFromNetwork();
                if (changed) repaint();
            });
            ntPoller.setRepeats(true);
            ntPoller.start();
        } catch (Throwable t) {
            System.err.println("NT init failed: " + t.getMessage());
            ballsXSub = null;
            ballsYSub = null;
            if (ntPoller != null) ntPoller.stop();
            ntPoller = null;
        }
    }

    // Read arrays from NetworkTables, convert robot-relative coords to field coords, update balls.
    // Returns true if we replaced the ball list with any valid NT detections.
    private boolean updateBallsFromNetwork() {
        if (ballsXSub == null || ballsYSub == null) return false;
        double[] xs;
        double[] ys;
        try {
            xs = ballsXSub.get();
            ys = ballsYSub.get();
        } catch (Throwable t) {
            return false;
        }
        if (xs == null || ys == null) return false;
        int n = Math.min(xs.length, ys.length);
        if (n == 0) return false;
        double rx = GoToBall.robotX;
        double ry = GoToBall.robotY;
        double rtheta = Math.toRadians(GoToBall.robotRot);
        double cos = Math.cos(rtheta);
        double sin = Math.sin(rtheta);
        java.util.List<FuelStruct> newBalls = new java.util.ArrayList<>();
        final double INCH_TO_M = 0.0254;
        for (int i = 0; i < n; i++) {
            double relX = xs[i] * INCH_TO_M;
            double relY = ys[i] * INCH_TO_M;
            double fx = rx + (cos * relX - sin * relY);
            double fy = ry + (sin * relX + cos * relY);
            if (fx >= fieldBoundMinX && fx <= fieldBoundMaxX && fy >= fieldBoundMinY && fy <= fieldBoundMaxY) {
                if (isValidPosition(fx, fy)) {
                    newBalls.add(new FuelStruct(fx, fy, 10));
                }
            }
        }
        if (!newBalls.isEmpty()) {
            vision_data.clear();
            vision_data.addAll(newBalls);
            return true;
        }
        return false;
    }
}