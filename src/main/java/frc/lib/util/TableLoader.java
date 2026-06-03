package frc.lib.util;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.*;
import java.io.File;

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
}