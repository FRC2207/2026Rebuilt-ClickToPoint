package frc.robot;

public class Ball {
    double xMeters, yMeters; // Field coordinates in meters
    int radius = 10; // Ball radius in pixels

    public Ball(double xMeters, double yMeters) {
        this.xMeters = xMeters;
        this.yMeters = yMeters;
    }

    // Convert meter coordinates to pixel coordinates for display
    int getPixelX() {
        return (int) (xMeters * GoToBallLine.PIXELS_PER_METER) + 25;
    }

    int getPixelY() {
        return (int) (yMeters * GoToBallLine.PIXELS_PER_METER) + 25;
    }

    public boolean contains(int px, int py) {
        int dx = getPixelX() - px;
        int dy = getPixelY() - py;
        return (dx * dx + dy * dy) <= (radius * radius);
    }
}
