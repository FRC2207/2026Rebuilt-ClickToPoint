package frc.robot;

import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;

class NavGrid {
    double fieldWidth, fieldHeight;
    double nodeSize;
    boolean[][] grid;

    public NavGrid(String jsonPath) {
        fieldWidth = 16.54; // Default from navgrid
        fieldHeight = 8.07; // Default from navgrid
        nodeSize = 0.3; // Default

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
        if (pos < 0)
            return 0;

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
        if (grid.length == 0)
            return true;

        // Convert meters to grid coordinates
        int gridX = (int) (xMeters / nodeSize);
        int gridY = (int) (yMeters / nodeSize);

        // Bounds check
        if (gridX < 0 || gridX >= grid[0].length || gridY < 0 || gridY >= grid.length) {
            return false;
        }
        boolean result = grid[gridY][gridX];
        return result;
    }
}
