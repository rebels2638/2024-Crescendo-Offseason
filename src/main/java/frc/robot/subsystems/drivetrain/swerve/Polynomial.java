package frc.robot.subsystems.drivetrain.swerve;

public class Polynomial {
    private int DEGREE;
    // intercept first
    private double[] COEFFICIENTS;

    public Polynomial(int DEGREE, double[] COEFFICIENTS) {
        this.DEGREE = DEGREE;
        this.COEFFICIENTS = COEFFICIENTS;
    }

    public double function(double[] x) {
        double result = COEFFICIENTS[0];
        for (int i = 1; i < COEFFICIENTS.length; i++) {
            int p = i % DEGREE;
            if (p == 0) {
                p = DEGREE;
            }

            result += COEFFICIENTS[i] * Math.pow(x[((i - 1) / DEGREE)], p);
        }

        return result;
    }

    // testing code
    // public static void main(String[] args) {
    //     Polynomial poly = new Polynomial(3, new double[] {4, 1, 2, 14, 2, 5, 4});
    //     System.out.println(poly.function(new double[] {6.5, 1.6}));

    // }

}
