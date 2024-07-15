package frc.robot.subsystems.drivetrain.swerve;

import java.util.ArrayList;

public class DriveFFController {
    ArrayList<double[]> points = new ArrayList<double[]>();
    public DriveFFController() {
        // drive meters per second, radians per second, mps error
        points.add(new double[] {0, 0, 0});
        points.add(new double[] {1, 0, 0});
        points.add(new double[] {2, 0, 0});
        points.add(new double[] {0, Math.PI, 0});
        points.add(new double[] {0, 2 * Math.PI, 0});
        points.add(new double[] {2, 2 * Math.PI, 0.45});
        points.add(new double[] {1, 2 * Math.PI, 0.212});
        points.add(new double[] {2, Math.PI, 0.344});
        points.add(new double[] {1, Math.PI, 0.25});
    }

    public double calculate(double imps, double irps) {
        double mps = Math.abs(imps);
        double rps = Math.abs(irps);
        for (int i = 0; i < points.size(); i++) {
            if (points.get(i)[0] == mps && points.get(i)[1] == rps) {
                return points.get(i)[2];
            }
        }

        double[] inputPoint = new double[] {mps, rps, 0};

        double[][] p = new double[4][3];
        for (int i = 0; i < 4; i++) {
            p[i] = new double[] {Integer.MAX_VALUE, Integer.MAX_VALUE, -1};
        }
        
        ArrayList<double[]> temp = points;
        for (int n = 0; n < 4; n++) {
            int index = 0;
            for(int i = 0; i < temp.size(); i++) {
                if (distance(temp.get(i), inputPoint) < distance(p[n], inputPoint)) {
                    boolean good = true;
                    for (int m = 0; m < n; m++) {
                        if (distance(p[m], temp.get(i)) > Math.sqrt(2) + 0.1) {
                            good = false;
                        }
                    }
                    if (good) {
                        p[n] = temp.get(i);
                        index = i;
                    }
                }
            }
            temp.remove(index);
        }
        
        double[] tr, tl, br, bl;
        tr = new double[] {-Integer.MAX_VALUE, -Integer.MAX_VALUE, 0};
        tl = new double[] {Integer.MAX_VALUE, -Integer.MAX_VALUE, 0};
        br = new double[] {-Integer.MAX_VALUE, Integer.MAX_VALUE, 0};
        bl = new double[] {Integer.MAX_VALUE, Integer.MAX_VALUE, 0};

        boolean trT = false;
        boolean tlT = false;
        boolean brT = false;
        boolean blT = false;


        for (int i = 0; i < 4; i++) {
            if (p[i][0] >= inputPoint[0] && p[i][1] >= inputPoint[1] && !trT) {
                tr = p[i];
                trT = true;
            }
            else if (p[i][0] <= inputPoint[0] && p[i][1] >= inputPoint[1] && !tlT) {
                tl = p[i];
                tlT = true;
            }
            else if (p[i][0] >= inputPoint[0] && p[i][1] <= inputPoint[1] && !brT) {
                br = p[i];
                brT = true;
            }
            else if (p[i][0] <= inputPoint[0] && p[i][1] <= inputPoint[1] && !blT) {
                bl = p[i];
                blT = true;
            }
        }
         
        double r1 = (tr[0] - inputPoint[0])/(tr[0] - tl[0]) * tl[2] + (inputPoint[0] - tl[0])/(tr[0] - tl[0]) * tr[2];
        double r2 = (br[0] - inputPoint[0])/(br[0] - bl[0]) * bl[2] + (inputPoint[0] - bl[0])/(br[0] - bl[0]) * br[2];
        double pf = (inputPoint[1] - br[1])/(tr[1] - br[1]) * r1 + (tr[1] - inputPoint[1])/(tr[1] - bl[1]) * r2;

        if (Double.isNaN(r1)) {
            return r2;
        }
        if (Double.isNaN(r2)) {
            return r1;
        }

        if (mps >= 0 && rps >= 0) {
            return pf;
        }

        if (mps >= 0 && rps <= 0) {
            return -pf;
        }
        
        if (mps <= 0 && rps >= 0) {
            return -pf;
        }

        return pf;

    }

    private double distance(double[] a, double[] b) {
        return Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow((a[1] - b[1]) / Math.PI, 2));
    }

    public static void main(String[] args) {
        DriveFFController driveFFController = new DriveFFController();

        System.out.println(driveFFController.calculate(1, 1));
    }
}