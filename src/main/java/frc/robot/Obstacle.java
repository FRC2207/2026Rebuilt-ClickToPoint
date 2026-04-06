package frc.robot;

class Obstacle {
    double xMeters, yMeters; // Field coordinates in meters
    double radiusMeters; // Radius in meters

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
