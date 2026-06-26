package frc.lib.util;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.*;

import org.littletonrobotics.junction.Logger;

public class TableLoader {

    public static NestedInterpolatingTreeMap<Double, Translation3d> loadFromCSV(String path) {
        NestedInterpolatingTreeMap<Double, Translation3d> table =
            new NestedInterpolatingTreeMap<>(
                (a, b, query) -> (query - a) / (b - a),
                Translation3d::interpolate
            );

        File file = new File(Filesystem.getDeployDirectory(), path);

        // DEBUG: print full path
        System.out.println("Attempting to load CSV from: " + file.getAbsolutePath());

        // Check if file exists
        if (!file.exists()) {
            Logger.recordOutput("TableLoader", false);
            return table;
        }
        Logger.recordOutput("TableLoader", true);
        System.out.println("CSV file found!");

        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line;

            br.readLine(); // skip header

            int count = 0;

            while ((line = br.readLine()) != null) {

                String[] tokens = line.split(",");
                if (tokens.length != 6) continue;

                double x = Double.parseDouble(tokens[0]);
                double y = Double.parseDouble(tokens[1]);
                double zX = Double.parseDouble(tokens[2]);
                double zY = Double.parseDouble(tokens[3]);
                double ignored = Double.parseDouble(tokens[4]);
                double zZ = Double.parseDouble(tokens[5]);

                table.put(x, y, new Translation3d(zX, zY, zZ));

                count++;
            }

            System.out.println("Loaded " + count + " shooting table entries");

        } catch (IOException e) {
            System.out.println("ERROR reading CSV:");
            e.printStackTrace();
        }

        return table;
    }
    /**
     * Loads a standard WPILib InterpolatingDoubleTreeMap from a CSV file deployed on the RIO.
     * Maps distance (col 0) to solutionVelocity (col 2).
     * * @param path The relative path inside the deploy directory (e.g., "tuning/shooter.csv")
     * @return A populated InterpolatingDoubleTreeMap
     */
    public static InterpolatingDoubleTreeMap loadDoubleMapFromCSV(String path) {
        InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
        File file = new File(Filesystem.getDeployDirectory(), path);

        System.out.println("Attempting to load Double map CSV from: " + file.getAbsolutePath());

        if (!file.exists()) {
            Logger.recordOutput("TableLoader/DoubleMap_" + path, false);
            return table;
        }
        Logger.recordOutput("TableLoader/DoubleMap_" + path, true);
        System.out.println("Double CSV file found!");

        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line;
            br.readLine(); // skip header row

            int count = 0;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty()) continue;

                String[] tokens = line.split(",");
                // Ensure there are enough columns to parse up to index 2
                if (tokens.length < 3) continue; 

                try {
                    double distance = Double.parseDouble(tokens[0]);      // Column 1
                    double velocity = Double.parseDouble(tokens[2]);      // Column 3 (solutionVelocity)

                    table.put(distance, velocity);
                    count++;
                } catch (NumberFormatException e) {
                    System.out.println("Warning: Skipped malformed row in CSV: " + line);
                }
            }

            System.out.println("Loaded " + count + " double map entries from CSV.");

        } catch (IOException e) {
            System.out.println("ERROR reading Double CSV:");
            e.printStackTrace();
        }

        return table;
    }
}