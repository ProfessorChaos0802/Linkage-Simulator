package main;

import java.util.ArrayList;

/**
 * Link object used to construct and animate a linkage
 */
public class Link{
    private boolean isGear; // boolean denoting if the link is a gear
    private boolean isExt; // boolean denoting if the gear is an external gear

    private int Fpin; // Pin the force is acting on
    private int NB; // Number of pins on the boundary
    private int NS; // Number of slot pins

    private double theta; // angle between the local x-axis and the global x-axis
    private double P; // Pitch of gear
    private double m; // mass of link
    private double I; // mass moment of Intertia of link

    private int[] colors = new int[3]; // color(1,:) => pins and boundary
                                                              // color(2,:) => fill
                                                              // color(3,:) => slot

    private double[] x0 = new double[2]; // link origin
    private double[] q = new double[3]; // generalized coordinates for the link origin {x0 = q(1:2, :), theta = q(3,:)}
    private double[] F; // force acting on the link

    private ArrayList<Double>[] pins = new ArrayList[2]; // local x,y coordinates of pins
    private ArrayList<int[]> slotPins = new ArrayList<int[]>(); // All of the pins needed for a making a slot
    /**
     * Constructor if the link does not contain a slot
     *
     * @param x0 - link origin
     * @param theta - angle between the local and global x-axis
     * @param pins - number of pins on the link
     * @param NB - number of external pins on the link
     * @param colors - colors used for plotting
     */
    public Link(double[] x0, double theta, ArrayList<Double>[] pins, int NB, int[] colors){
        this.q[0] = x0[0];
        this.q[1] = x0[1];
        this.q[2] = theta;
        this.x0 = x0;
        this.pins = pins;
        this.NB = NB;
        this.colors = colors;
        this.NS = 0;
        this.isGear = false;
        this.isExt = true;
        this.m = 0;
        this.I = 0;
        this.F = new double[]{0, 0};
        this.Fpin = 0;

    }

    /**
     * Constructor if the link does contain a slot
     *
     * @param x0 - link origin
     * @param theta - angle between the local and global x-axis
     * @param pins - number of pins on the link
     * @param NB - number of external pins on the link
     * @param colors - colors used for plotting
     * @param slotPins - array of pins that are connected togeter using a slot
     */
    public Link(double[] x0, double theta, ArrayList<Double>[] pins, int NB, int[] colors, ArrayList<int[]> slotPins){
        this.q[0] = x0[0];
        this.q[1] = x0[1];
        this.q[2] = theta;
        this.x0 = x0;
        this.pins = pins;
        this.NB = NB;
        this.colors = colors;
        this.NS = slotPins.size();
        this.slotPins = slotPins;
        this.isGear = false;
        this.isExt = true;
        this.m = 0;
        this.I = 0;
        this.F = new double[]{0, 0};
        this.Fpin = 0;
    }

    /**
     * Translates and rotates all the points on the link, so that the link location
     * can be properly updated
     *
     * @return - New points of the link
     */
    public ArrayList<Double>[] LinkTranslate(){

        ArrayList<Double>[] P = RotationMatrixMultiply(this.pins);

        for (int i = 0; i < P[0].size(); i ++) {
            P[0].set(i, P[0].get(i) + this.q[0]);
            P[1].set(i, P[1].get(i) + this.q[1]);
        }

        return P;
    }

    /**
     * Returns the vectors sP, tP, rP in the form:
     *      result = {sP
     *                tP
     *                rP}
     *
     * sP -> location of the pin with respect to the local origin
     * tP -> vector perpendicular to sP
     * rP -> global coordinates of the selected pin
     *
     * @param pin - the pin from which you want sP, tP, and rP calculated from
     * @return
     */
    public double[][] LinkVector(int pin){
        ArrayList<Double>[] desiredPin = new ArrayList[1];
        desiredPin[0].add(pins[0].get(pin));
        desiredPin[1].add(pins[1].get(pin));

        ArrayList<Double>[] sp = RotationMatrixMultiply(desiredPin); // Vector from local origin to pin

        double[] sP = new double[]{sp[0].get(0), sp[0].get(1)}; // Refactored sp vector
        double[] tP = new double[]{-sP[1], sP[0]}; // Vector perpendicular to pin
        double[] rP = new double[]{(-sP[0] + q[0]), (sP[1] + q[1])}; // Global Coordinates of pin

        return new double[][]{sP, tP, rP};
    }

    /**
     * Returns useful set of vectors for a translated and rotated slot in the form:
     *
     * result = {rP1
     *           tP1
     *           v
     *           w}
     *
     * @param slot - the number of the slot you want information about
     * @return - informative slot vectors
     */
    public double[][] SlotAngle(int slot){
        double[][] slot1 = LinkVector(slotPins.get(slot)[0]);
        double[][] slot2 = LinkVector(slotPins.get(slot)[1]);

        double[] v = new double[2];
        v[0] = slot2[2][0] - slot1[2][0];
        v[1] = slot2[2][1] - slot1[2][1];

        double normV = Norm(v);
        v[0] = v[0] / normV;
        v[1] = v[1] / normV;

        double[] w = new double[]{-v[1], v[0]};

        return new double[][]{slot1[2], slot1[1], v, w};
    }

    /**
     * Returns the current angle of the link
     * @return
     */
    public double getTheta(){
        return q[2];
    }

    /**
     * Returns the x-, y- coordinates of the desired pin
     *
     * @param pin - the pin you want the coordinates of
     * @return
     */
    public double[] getPin(int pin){ return new double[]{pins[0].get(pin), pins[1].get(pin)};}

    /**
     * Returns the local origin of the link
     * @return
     */
    public double[] getOrigin(){ return new double[]{x0[0], x0[1]}; }

    /**
     * Returns the q vector of the link
     *
     * @return
     */
    public double[] getQ(){return q;}

    /**
     * Returns the colors used for plotting the link
     * @return
     */
    public int[] getColors(){ return colors;}

    /**
     * Returns the list of slotPins of the link
     * @return
     */
    public ArrayList<int[]> getSlotPins(){ return slotPins; }

    /**
     * Returns the mass of the link in kg
     * @return
     */
    public double getMass(){return m;}

    /**
     * Returns the force acting on the link in N
     * @return
     */
    public double[] getF(){return F;}

    /**
     * Returns the inertia of the link about its center of mass in kg*m^2
     * @return
     */
    public double getInertia(){return I;}

    /**
     * Returns the pin the force is acting on
     * @return
     */
    public int getFpin(){return Fpin;}

    /**
     * Sets the q vector of the link to the provided q vector
     * @param newQ - new q vector of the link
     */
    public void setQ(double[] newQ){
        q = newQ;
    }

    /**
     * Performs rotates the provided matrix using the rotation matrix with the link
     * q angle
     *
     * @param b - second matrix in the multiplication
     * @return result - result of the matrix multiplication
     */
    public ArrayList<Double>[] RotationMatrixMultiply(ArrayList<Double>[] b){
        ArrayList<Double>[] result = new ArrayList[2];
        result[0] = new ArrayList<Double>();
        result[1] = new ArrayList<Double>();

        // Build rotation matrix
        double c = Math.cos(this.q[2]);
        double s = Math.sin(this.q[2]);

        double[][] a = new double[][]{{c, -s}, {s, c}};

        // Loop over the rows of the first matrix
        for (int i = 0; i < a.length; i++){
            // Loop over the columns of the second matrix
            for (int j = 0; j < b[1].size(); j++){
                result[i].add(j, a[i][0]*b[0].get(j) + a[i][1]+b[1].get(j));
            }
        }

        return result;
    }

    /**
     * Calculates the norm of a vector
     *
     * @param v - vector you want the norm of
     * @return - norm of the vector
     */
    public double Norm(double[] v){
        double sum = 0;

        for(double num:v){
            sum += Math.pow(num, 2);
        }

        return Math.sqrt(sum);
    }

}