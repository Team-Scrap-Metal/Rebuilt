package frc.robot.util;

public class BestFitLine {
    private final double m_slope;
    private final double m_intercept;

    public BestFitLine (double[] x, double[] y) {
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
        m_slope = (sumXY - n * xBar * yBar) / (sumX2 - n * xBar * xBar);
        m_intercept = yBar - m_slope * xBar;
    }

    public double getBestFit(double x) {
        return m_slope * x + m_intercept;
    }
}
