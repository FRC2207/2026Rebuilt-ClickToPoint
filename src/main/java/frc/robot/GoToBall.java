package frc.robot;

import javax.swing.*;
import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.io.File;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class GoToBall extends SubsystemBase{
    // 2026 FRC field dimensions (in meters)
    static final double FIELD_LENGTH = 16.46; // 54 feet
    static final double FIELD_WIDTH = 8.23;   // 27 feet
    static final int PIXELS_PER_METER = 50;   // Scale factor for display
    static final double robotX = 1.0;
    static final double robotY = 4.3;
    static final double robotRot = 36.0;
    public static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public static NetworkTable table = inst.getTable("VisionData");
    public static NetworkTable poseTable = inst.getTable("AdvantageKit");
    public static StructArraySubscriber<FuelStruct> fuelSub;
    public static StructSubscriber<Pose2d> poseSub;
    public static Pose2d currentPose = new Pose2d(0, 0, new Rotation2d());
    public static StructPublisher<Pose2d> clickPub;
    public void periodic(){
        currentPose = GoToBall.poseSub.get();
    }
    
    public static void main(String[] args) {
        inst.startClient4("GoToBallViewer");          // set a client identity name
        inst.setServer("localhost", 5810);     // IMPORTANT: if you are running sim, serverName should be localhost. If not, it should b your team number(10.22.7.2)
        fuelSub = table.getStructArrayTopic("vision_data", FuelStruct.struct).subscribe(new FuelStruct[0]);
        poseSub = poseTable.getStructTopic("RealOutputs/Odometry/Robot", Pose2d.struct).subscribe(new Pose2d());
        clickPub = table.getStructTopic("TargetPoseClicked", Pose2d.struct).publish();
        System.out.println(inst.isConnected());
        
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

        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("FRC 2026 Field Pose Picker");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
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


class BallPanel extends JPanel {
    java.util.List<FuelStruct> vision_data;
    BufferedImage fieldImage;
    double fieldBoundMinX = 0, fieldBoundMaxX = 16.46;
    double fieldBoundMinY = 0, fieldBoundMaxY = 8.23;
    double pixelsPerMeterX, pixelsPerMeterY;
    FuelStruct hoveredBall = null;

    public BallPanel() {
        vision_data = new java.util.ArrayList<>();
        try {
            fieldImage = ImageIO.read(new File("src/main/java/frc/robot/f5h5pjh7whrmr0cwb1v9zgfp5r_result_0.png")); // Name of field image with path
            int w = fieldImage.getWidth(); // Width of the image
            int h = fieldImage.getHeight(); // Height of the image
            BufferedImage rotated = new BufferedImage(h, w, fieldImage.getType());
            Graphics2D g2 = rotated.createGraphics(); // Basically gives us permissions to "draw" on the image-and rotate it
            g2.translate(h, 0); // Zero's out the offset before rotating to prevent the field image being off screen
            g2.rotate(Math.PI / 2); // Since g2.rotate is in radians, we do pi/2 to get 90 degrees instead of putting a rounded decimal
            g2.drawImage(fieldImage, 0, 0, null); // Draws the image onto the JFrame
            g2.dispose();
            fieldImage = rotated;
            pixelsPerMeterX = fieldImage.getWidth() / GoToBall.FIELD_LENGTH;
            pixelsPerMeterY = fieldImage.getHeight() / GoToBall.FIELD_WIDTH;
            detectFieldBoundaries();
        } catch (Exception e) {
            System.err.println("Failed to load field image: " + e.getMessage());
            fieldImage = null;
            return;
        }
        // Timer to update vision_data from NetworkTables every 100ms - in this loop it does the math to convert aidan's bot relative coords to field relative
        javax.swing.Timer ntUpdateTimer = new javax.swing.Timer(100, evt -> {
            FuelStruct[] ballsRaw = GoToBall.fuelSub.get();
            System.out.println(ballsRaw.length);
            Pose2d robotPose = GoToBall.currentPose;
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
        // Convert bot-relative coordinates to field-relative using Robot.currentPose
        // Math:
        // X_field = robotX + cos(theta) * ball.x - sin(theta) * ball.y
        // Y_field = robotY + sin(theta) * ball.x + cos(theta) * ball.y
        java.util.List<FuelStruct> fieldRelativeBalls = new java.util.ArrayList<>();
        // Robot.currentPose must be public static in Robot.java
        Pose2d robotPose = GoToBall.currentPose;
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotTheta = robotPose.getRotation().getRadians();
        double cosTheta = Math.cos(robotTheta);
        double sinTheta = Math.sin(robotTheta);
        for (FuelStruct ball : vision_data) {
            double fieldX = robotX + cosTheta * ball.x - sinTheta * ball.y;
            double fieldY = robotY + sinTheta * ball.x + cosTheta * ball.y;
            fieldRelativeBalls.add(new FuelStruct((float) fieldX, (float) fieldY));
        }
        vision_data = fieldRelativeBalls;
        addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) { // Adds method for when the mouse is pressed
                int panelWidth = getWidth();
                int panelHeight = getHeight();
                double imagePixelsPerMeterX = panelWidth / GoToBall.FIELD_WIDTH;
                double imagePixelsPerMeterY = panelHeight / GoToBall.FIELD_LENGTH;
                double clickXMeters = e.getX() / imagePixelsPerMeterX;
                double clickYMeters = e.getY() / imagePixelsPerMeterY;
                for (FuelStruct ball : vision_data) {
                    double dx = ball.x - clickXMeters;
                    double dy = ball.y - clickYMeters;
                    double distMeters = Math.sqrt(dx * dx + dy * dy);
                    if (distMeters < 0.2) {
                        double originalX = ball.y;
                        double originalY = ball.x;
                        Pose2d BallPose2d = new Pose2d(originalX, originalY, new Rotation2d()); // Makes the new Pose2d
                        System.out.println(BallPose2d.toString()); // Prints the target pose
                        GoToBall.clickPub.set(BallPose2d); // Publishes the Pose2d to NT
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
                double imagePixelsPerMeterX = panelWidth / GoToBall.FIELD_WIDTH;
                double imagePixelsPerMeterY = panelHeight / GoToBall.FIELD_LENGTH;
                double mouseXMeters = e.getX() / imagePixelsPerMeterX;
                double mouseYMeters = e.getY() / imagePixelsPerMeterY;
                FuelStruct newHoveredBall = null;
                for (FuelStruct ball : vision_data) {
                    double dx = ball.x - mouseXMeters;
                    double dy = ball.y - mouseYMeters;
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
    
    private void detectFieldBoundaries() { // Detects field boundaries, kinda broken most likely will change soon
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
    protected void paintComponent(Graphics g) { // This draws all of the graphical stuff
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        if (fieldImage != null) {
            int panelWidth = getWidth();
            int panelHeight = getHeight();
            g2d.drawImage(fieldImage, 0, 0, panelWidth, panelHeight, this);
            double panelPixelsPerMeterX = panelWidth / GoToBall.FIELD_WIDTH;
            double panelPixelsPerMeterY = panelHeight / GoToBall.FIELD_LENGTH;
            for (FuelStruct ball : vision_data) {
                int pixelX = (int)(ball.x * panelPixelsPerMeterX);
                int pixelY = (int)(ball.y * panelPixelsPerMeterY);
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
}