package frc.robot.util;

import Jama.Matrix;
import Jama.QRDecomposition;

// File found at 
// https://algs4.cs.princeton.edu/14analysis/MultipleLinearRegression.java


/**
 *  The {@code MultipleLinearRegression} class performs a multiple linear regression
 *  on an set of <em>N</em> data points using the model
 *  <em>y</em> = &beta;<sub>0</sub> + &beta;<sub>1</sub> <em>x</em><sub>1</sub> + ... +
    &beta;<sub><em>p</em></sub> <em>x<sub>p</sub></em>,
 *  where <em>y</em> is the response (or dependent) variable,
 *  and <em>x</em><sub>1</sub>, <em>x</em><sub>2</sub>, ..., <em>x<sub>p</sub></em>
 *  are the <em>p</em> predictor (or independent) variables.
 *  The parameters &beta;<sub><em>i</em></sub> are chosen to minimize
 *  the sum of squared residuals of the multiple linear regression model.
 *  It also computes the coefficient of determination <em>R</em><sup>2</sup>.
 *
 *  @author Robert Sedgewick
 *  @author Kevin Wayne
 */
public class MultipleLinearRegression {
    private final Matrix beta;  // regression coefficients
    private double sse;         // sum of squared
    private double sst;         // sum of squared

   /**
     * Performs a linear regression on the data points {@code (y[i], x[i][j])}.
     * @param  x the values of the predictor variables
     * @param  y the corresponding values of the response variable
     * @throws IllegalArgumentException if the lengths of the two arrays are not equal
     */
    public MultipleLinearRegression(double[][] x, double[] y) {
        if (x.length != y.length) {
            throw new IllegalArgumentException("matrix dimensions don't agree");
        }

        // number of observations
        int n = y.length;

        Matrix matrixX = new Matrix(x);

        // create matrix from vector
        Matrix matrixY = new Matrix(y, n);

        // find least squares solution
        QRDecomposition qr = new QRDecomposition(matrixX);
        beta = qr.solve(matrixY);


        // mean of y[] values
        double sum = 0.0;
        for (int i = 0; i < n; i++)
            sum += y[i];
        double mean = sum / n;

        // total variation to be accounted for
        for (int i = 0; i < n; i++) {
            double dev = y[i] - mean;
            sst += dev*dev;
        }

        // variation not accounted for
        Matrix residuals = matrixX.times(beta).minus(matrixY);
        sse = residuals.norm2() * residuals.norm2();

    }

   /**
     * Returns the least squares estimate of &beta;<sub><em>j</em></sub>.
     *
     * @param  j the index
     * @return the estimate of &beta;<sub><em>j</em></sub>
     */
    public double beta(int j) {
        return beta.get(j, 0);
    }

   /**
     * Returns the coefficient of determination <em>R</em><sup>2</sup>.
     *
     * @return the coefficient of determination <em>R</em><sup>2</sup>,
     *         which is a real number between 0 and 1
     */
    public double R2() {
        return 1.0 - sse/sst;
    }
}
