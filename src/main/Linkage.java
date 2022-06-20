package main;

import java.util.ArrayList;

public class Linkage {
    private Link[] links;
    private Constraint[] constraints;

    private int NL;
    private int NC;

    private double[] q;
    private double[] qdot;
    private double[] qddot;
    private double[] lambda;

    /**
     * Constructor for the Linkage class, takes only a list of links and constraints
     *
     * @param links - Links in the linkages
     * @param constraints - Constraints on how the links are linked together
     */
    public Linkage(Link[] links, Constraint[] constraints){
        this.links = links;
        this.constraints = constraints;
        NL = links.length;
        NC = constraints.length;

        q = new double[3*NL];
        qdot = new double[3*NL];
    }

    /**
     * Populates the q vector of the linkage with the q vectors
     * of all the links within the linkage
     */
    public double[] Links2Gen(){
        for (int i = 0; i < 3*NL; i++){
            q[i] = links[i/3].getQ()[i % 3]; // Set the q vector of the linkage equal to the q vector of all the links
        }

        return q;
    }

    /**
     * Updates the q values of each link with the current guess of q
     */
    public void Gen2Links(double[] q){
        for (int i = 0; i < NL; i++){
            double[] newQ = new double[]{q[i], q[i+1], q[i+2]};
            links[i].setQ(newQ);
        }
    }

    /**
     * Calculates the phi vector for the linkage based on the current guess of q
     *
     * @param q - current guess of link positions
     * @return - the calculated phi vector
     */
    public ArrayList<Double> calcPhi(double[] q){
        Gen2Links(q); // Transfer the current guess to link coords
        ArrayList<Double> phi = new ArrayList<Double>();

        for (int i = 0; i < NC; i++){
            phi = constraints[i].makePhi(this, phi);
        }

        return phi;
    }

    /**
     * Calculates the jacobian matrix using the current guess of q
     *
     * @param q - current guess of link coords
     * @return - jacobian matrix
     */
    public ArrayList<ArrayList<Double>> calcJac(double[] q){
        Gen2Links(q); // Transfer current guess to link coords
        ArrayList<ArrayList<Double>> jac = new ArrayList<>();

        for (int i = 0; i < NC; i++){
            jac = constraints[i].makeJac(this, jac);
        }

        return jac;
    }

    /**
     * Calculates the nu vector of the linkage
     *
     * @return - Nu vector
     */
    public ArrayList<Double> calcNu(){
        ArrayList<Double> nu = new ArrayList<Double>();

        for (int i = 0; i < NC; i++){
            nu = constraints[i].makeNu(nu);
        }

        return nu;
    }

    /**
     * Calculates the gamma vector for the current linkage
     *
     * @param qdot - current guess of velocities
     * @return - Gamma vector
     */
    public ArrayList<Double> calcGamma(double[] qdot){
        ArrayList<Double> gamma = new ArrayList<Double>();

        for (int i = 0; i < NC; i++){
            gamma = constraints[i].makeGamma(gamma, qdot, this);
        }

        return gamma;
    }

    /**
     * Calculates the M and Q matrices used in the force analysis
     *
     * @return - a 3D vector containing the M and Q matrices
     */
    public double[][][] calcMQ(){
        int n = 3*NL;

        double[][] m = new double[n][n];
        double[][] q = new double[1][n];

        for (int i = 0; i < NL; i++){
            Link l = links[i];

            for (int j = 3*i; j < 3*i + 3; j++){
                for (int k = 0; k < n; k++){
                    if(j == k && j != 3*i + 2){
                        m[j][k] = l.getMass();
                    }else if(j == k && j == 3*i + 2){
                        m[j][k] = l.getInertia();
                    }else{
                        m[j][k] = 0.0;
                    }
                }
            }

            double[] linkForce = l.getF();
            double[][] f = new double[][]{new double[]{linkForce[0]},
                                          new double[]{linkForce[1]}};
            double[][] tL = new double[][]{l.LinkVector(l.getFpin())[1]};

            q[0][0] = f[0][0];
            q[0][1] = f[0][1];
            q[0][2] = matrixMultiply(tL, f)[0][0];
        }

        return new double[][][]{m, q};
    }

    public void LinkageSolve(){
        double[] q = Links2Gen();
        ArrayList<Double> phi;
        ArrayList<ArrayList<Double>> jac;

        int maxIter = 20;
        double tol = 1e-6;
        int i = 1;

        while (i <= maxIter){
            phi = calcPhi(q);
            jac = calcJac(q);


            if( absMax(phi) < tol){
                Gen2Links(q);
                this.q = q;
                jac = calcJac(q);
                ArrayList<Double> nu = calcNu();


            }
        }
    }

    /**
     * Returns the specific link you want, 0-based indexing access
     * @param link - which link you want in the array (i.e. 0, 1, 2...)
     * @return - the link you queried
     */
    public Link getLink(int link){
        return links[link];
    }

    /**
     * Returns the number of links in the linkage
     * @return
     */
    public int numLinks(){
        return NL;
    }

    /**
     * Performs matrix multiplication on the given matrices
     *
     * @param a - First matrix to be multiplied
     * @param b - Second matrix to be multiplied
     * @return - result of the matrix multiplication
     */
    public double[][] matrixMultiply(double[][] a, double[][] b){
        if(a[0].length != b.length){
            throw new UnsupportedOperationException(String.format("Matrices cannot be multiplied together because Matrix A is %d x %d and Matrix B is %d x %d", a.length, a[0].length, b.length, b[0].length));
        }

        double[][] result = new double[a.length][b[0].length];
        double sum = 0;

        for (int i = 0; i < a.length; i++){
            sum = 0;
            for (int j = 0; j < b[0].length; j++){
                for (int k = 0; k < a[0].length; k++){
                    sum += a[i][k] + b[k][j];
                }
                result[i][j] = sum;
            }
        }

        return result;
    }

    /**
     * Takes the absolute value of all the numbers in the provided vector and
     * then returns the largest number in the vector.
     *
     * @param vec - vector in which you want the number with the largest magnitude
     * @return - the number in vec with the largest magnitude
     */
    public double absMax(ArrayList<Double> vec){
        ArrayList<Double> allPositives = new ArrayList<Double>();
        double max = 0;

        // Take the absolute value of vec and finds the number with the largest
        // magnitude
        for (int i = 0; i < vec.size(); i++){
            if (vec.get(i) < 0){
                allPositives.add(i, -1*vec.get(i));
            }else{
                allPositives.add(i, vec.get(i));
            }

            if(allPositives.get(i) > max){
                max = allPositives.get(i);
            }
        }

        return max;
    }


}
