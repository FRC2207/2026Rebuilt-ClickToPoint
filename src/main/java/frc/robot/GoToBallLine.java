package frc.robot;

import javax.swing.*;
import java.awt.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Main entry and shared NetworkTables state for the ClickToPoint viewer.
 */
public class GoToBallLine {
    // 2026 FRC field dimensions (in meters)
    static final double FIELD_HEIGHT = 16.46; // 54 feet
    static final double FIELD_WIDTH = 8.23; // 27 feet
    static final double FIELD_LENGTH_MARGIN = 0.3;
    static final double FIELD_WIDTH_MARGIN = 0.2;
    static final double IMAGE_WIDTH_METERS = FIELD_WIDTH + FIELD_WIDTH_MARGIN * 2;
    static final double IMAGE_HEIGHT_METERS = FIELD_HEIGHT + FIELD_LENGTH_MARGIN * 2;
    static final int PIXELS_PER_METER = 50; // Scale factor for display
    static final double ROBOT_WIDTH = 1.080;
    static final double ROBOT_HIEGHT = 0.705;

    public static NetworkTableInstance inst;
    public static NetworkTable table;
    public static NetworkTable poseTable;
    public static NetworkTable presetsTable;
    // Publishers for preset buttons (initialized in main)
    public static BooleanPublisher trenchLeft;
    public static BooleanPublisher trenchRight;
    public static BooleanPublisher shootLeft;
    public static BooleanPublisher shootMiddle;
    public static BooleanPublisher shootRight;
    public static BooleanPublisher outpost;
    public static StructArraySubscriber<FuelStruct> fuelSub;
    public static StructSubscriber<Pose2d> poseSub;
    public static Pose2d currentPose = new Pose2d(0, 0, new Rotation2d());
    public static edu.wpi.first.networktables.BooleanSubscriber isRedAllianceSub;
    public static boolean simActive = false;

    public static void main(String[] args) {
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("VisionData");
        poseTable = inst.getTable("AdvantageKit");
        presetsTable = inst.getTable("PresetTriggers");
        inst.startClient4("GoToBallViewer");
        inst.setServer("10.22.7.2", 5810); // IMPORTANT: if you are running sim, serverName should be localhost. If not, it should be your team number(10.22.7.2)
        fuelSub = table.getStructArrayTopic("vision_data", FuelStruct.struct).subscribe(new FuelStruct[0]);
        poseSub = poseTable.getStructTopic("RealOutputs/Odometry/Robot", Pose2d.struct).subscribe(new Pose2d());

        // Initialize preset publishers (make them static so UI code can access them)
        trenchLeft = presetsTable.getBooleanTopic("Trench Left").publish();
        trenchRight = presetsTable.getBooleanTopic("Trench Right").publish();
        shootLeft = presetsTable.getBooleanTopic("ShootL").publish();
        shootMiddle = presetsTable.getBooleanTopic("ShootM").publish();
        shootRight = presetsTable.getBooleanTopic("ShootR").publish();
        outpost = presetsTable.getBooleanTopic("Outpost").publish();

        // Default values
        trenchLeft.set(false);
        trenchRight.set(false);
        shootLeft.set(false);
        shootMiddle.set(false);
        shootRight.set(false);
        outpost.set(false);

        isRedAllianceSub = inst.getTable("FMSInfo").getBooleanTopic("IsRedAlliance").subscribe(false);

        SwingUtilities.invokeLater(() -> {
            GraphicsEnvironment ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
            GraphicsDevice[] screenDevices = ge.getScreenDevices();

            JFrame frame = new JFrame("FRC Live Field Click To Point");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setSize((int) (IMAGE_HEIGHT_METERS * PIXELS_PER_METER), (int) (IMAGE_WIDTH_METERS * PIXELS_PER_METER));
            frame.setLocationRelativeTo(null);
            BallPanel panel = new BallPanel();
            frame.add(panel);

            Image icon = Toolkit.getDefaultToolkit().getImage("src/main/java/frc/robot/ClickToPoint Logo.png");
            frame.setIconImage(icon);

            if (screenDevices.length > 1) {
                Rectangle bounds = screenDevices[1].getDefaultConfiguration().getBounds();
                frame.setLocation(bounds.x, bounds.y);
                frame.setExtendedState(Frame.MAXIMIZED_BOTH);
                frame.setUndecorated(true);
            }
            frame.setVisible(true);
        });
    }

}