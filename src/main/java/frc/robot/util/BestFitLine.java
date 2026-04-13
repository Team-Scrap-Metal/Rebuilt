package frc.robot.util;

public class BestFitLine {
    public static void main(String[] args) {
        double[] x = {1, 2, 3, 4, 5};
        double[] y = {2, 4, 5, 4, 5};
        
        int n = x.length;
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

        for (int i = 0; i < n; i++) {
            sumX += x[i];
            sumY += y[i];
            sumXY += x[i] * y[i];
            sumX2 += x[i] * x[i];
        }

        double xBar = sumX / n;
        double yBar = sumY / n;

        // Calculate slope (m) and intercept (b)
        double slope = (sumXY - n * xBar * yBar) / (sumX2 - n * xBar * xBar);
        double intercept = yBar - slope * xBar;

        System.out.println("Equation: y = " + slope + "x + " + intercept);
    }
}
