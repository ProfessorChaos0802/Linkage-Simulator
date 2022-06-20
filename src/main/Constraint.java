package main;

import java.util.ArrayList;

public class Constraint {
    private String type; // Type of constraint
    private int link1; // First link in constraint
    private int pin1; // Constrained pin on first link
    private int link2; // Second pin in constraint
    private int pin2; // Constrained pin on second link
    private double theta0; // Initial displacement of the link
    private double omega; // Rotational velocity
    private int slot; // Number of slot on the link
    private double rho; // Gear ratio
    private double offset; // Angular offset between gears
    private double sliderAngle; // Angle between the slider and the coupler

    private double[] rhoP; // Gear ratios for planetary gears
    private double[] offsetP; // Offset for planetary gears

    /**
     * Constructor for a Ground Constraint
     *
     * TODO: Handle IllegalArgumentException
     *
     * @param type  - Name of the constraint
     * @param link1 - Link to be pinned to the ground
     * @param pin1  - Origin of the link
     */
    public Constraint(String type, int link1, int pin1) {
        this.type = type;

        switch (type) {
            case "Gnd":
                this.link1 = link1;
                this.pin1 = pin1;
            default:
                throw new IllegalArgumentException();
        }
    }

    /**
     * Constructor for a PinPin and PinSlot constraint
     *
     * TODO: Handle IllegalArgumentException
     *
     * @param type  - Name of the constraint
     * @param link1 - First link to be constrained
     * @param pin1  - Pin to be constrained on link 1
     * @param link2 - Second link to be constrained
     * @param pin2  - Pin to be constrained for PinPin OR slot to be constrained for PinSlot
     */
    public Constraint(String type, int link1, int pin1, int link2, int pin2) {
        this.type = type;

        switch (type) {
            case "PinPin":
                this.link1 = link1;
                this.pin1 = pin1;
                this.link2 = link2;
                this.pin2 = pin2;
            case "PinSlot":
                this.link1 = link1;
                this.pin1 = pin1;
                this.link2 = link2;
                slot = pin2;
            default:
                throw new IllegalArgumentException();
        }
    }

    /**
     * Constructor for a DriveRot and Gear constraint
     *
     * TODO: Handle IllegalArgumentException
     *
     * @param type   - Name of the constraint
     * @param link1  - First link to be constrained
     * @param link2  - Second link to be constrained
     * @param theta0 - Initial displacement of the link for DriveRot OR gear ratio for Gear
     * @param omega  - Angular speed for DriveRot OR initial angular displacement for Gear
     */
    public Constraint(String type, int link1, int link2, double theta0, double omega) {
        this.type = type;

        switch (type) {
            case "DriveRot":
                this.link1 = link1;
                this.link2 = link2;
                this.theta0 = theta0;
                this.omega = omega;
            case "Gear":
                this.link1 = link1;
                this.link2 = link2;
                this.rho = theta0;
                this.offset = omega;
            default:
                throw new IllegalArgumentException();
        }
    }

    /**
     * Constructor for a Planetary constraint
     *
     * TODO: Handle IllegalArgumentException
     *
     * @param type   - Name of the constraint
     * @param link1  - First link to be constrained
     * @param link2  - Second link to be constrained
     * @param rho    - Planetary gear ratios
     * @param offset - Angular offset between links
     */
    public Constraint(String type, int link1, int link2, double[] rho, double[] offset) {
        this.type = type;

        switch (type) {
            case "Planetary":
                this.link1 = link1;
                this.link2 = link2;
                rhoP = rho;
                offsetP = offset;
            default:
                throw new IllegalArgumentException();
        }
    }

    /**
     * Fills in the Phi vector based on the type of constraint
     *
     * TODO: Handle NullPointerException
     * TODO: Handle UnsupportedOperationException
     *
     * @param linkage - passed in linkage from projcet
     * @param phi - partially completed Phi vector
     * @return - a more complete phi vector
     */
    public ArrayList<Double> makePhi(Linkage linkage, ArrayList<Double> phi) {
        // Check that phi is valid
        if (phi == null) {
            throw new NullPointerException("Phi Vector is null");
        }

        if(linkage == null){
            throw new NullPointerException("Linkage is undefined");
        }

        // Instantiate useful variables
        Link l1;
        Link l2;

        double[][] link1;
        double[][] link2;
        double[] rP;
        double[] rQ;
        double[] w;
        double[] u;

        switch (type) {
            // Phi value for ground constraint
            case "Gnd":
                l1 = linkage.getLink(this.link1);
                link1 = l1.LinkVector(pin1);
                rP = link1[2];

                phi.add(rP[0]);
                phi.add(rP[1]);
                phi.add(l1.getTheta());
                return phi;

            // Phi value for PinPin
            case "PinPin":
                l1 = linkage.getLink(this.link1);
                l2 = linkage.getLink(this.link2);

                link1 = l1.LinkVector(pin1);
                link2 = l2.LinkVector(pin2);

                rP = link1[2];
                rQ = link2[2];

                phi.add(rP[0] - rQ[0]);
                phi.add(rP[1] - rQ[1]);
                return phi;

            // Phi value for DriveRot
            case "DriveRot":
                l1 = linkage.getLink(this.link1);
                l2 = linkage.getLink(this.link2);
                phi.add(l1.getTheta() - l2.getTheta() - theta0);
                return phi;

            // Phi value for PinSlot
            case "PinSlot":
                l1 = linkage.getLink(this.link1);
                l2 = linkage.getLink(this.link2);
                link1 = l1.LinkVector(pin1);
                link2 = l2.SlotAngle(slot);

                rP = link1[2];
                rQ = link2[0];
                w = link2[3];

                u = new double[]{(rP[0] - rQ[0]), (rP[1] - rQ[1])};

                phi.add(w[0] * u[0] + w[1] * u[1]);
                return phi;

            // Phi value for Gear
            case "Gear":
                l1 = linkage.getLink(this.link1);
                l2 = linkage.getLink(this.link2);
                phi.add(l2.getTheta() + rho * l1.getTheta() - offset);
                return phi;

            // Phi value for Planetary
            case "Planetary":
                l1 = linkage.getLink(this.link1);
                l2 = linkage.getLink(this.link2);
                double lambda = rhoP[0] * (offsetP[0] + l1.getTheta()) + rhoP[1] * (offsetP[1] + l2.getTheta() - Math.PI);
                link1 = l1.LinkVector(pin1);
                link2 = l2.LinkVector(pin2);

                rP = link1[2]; // Sun
                rQ = link2[2]; // Planet

                w = new double[]{-Math.sin(lambda), Math.cos(lambda)};
                u = new double[]{(rP[0] - rQ[0]), (rP[1] - rQ[1])};

                phi.add(w[0] * u[0] + w[1] * u[1]);
                return phi;

            default:
                throw new UnsupportedOperationException("Not a valid constraint");
        }
    }

    /**
     * Defines the Jacobian Matrix for the given linkage
     *
     * TODO: Handle UnsupportedOperationException
     * TODO: Handle NullPointerException
     *
     * @param linkage - The linkage being analyzed
     * @param jac - The Jacobian matrix we are building
     * @return
     */
    public ArrayList<ArrayList<Double>> makeJac(Linkage linkage, ArrayList<ArrayList<Double>> jac){
        if(jac == null){
            throw new NullPointerException("Jacobian Matrix is null");
        }

        if(linkage == null){
            throw new NullPointerException("Linkage is undefined");
        }

        Link link1 = linkage.getLink(this.link1);
        Link link2 = linkage.getLink(this.link2);

        double[][] linkP;
        double[][] linkQ;

        double[] tP;
        double[] tQ;
        double[] rP;
        double[] rQ;

        ArrayList<Integer> cols1 = new ArrayList<Integer>();
        ArrayList<Integer> cols2 = new ArrayList<Integer>();

        int dof = 3*linkage.numLinks();
        int n = jac.size();
        int c1 = 0;
        int c2 = 0;

        ArrayList[] j;
        ArrayList<Double> jacobian;

        for (int i = 0; i < 3; i++){
            cols1.add(i, 3*(this.link1 - 1) + i);
            cols2.add(i, 3*(this.link2 - 1) + i);
        }

        switch (type){

            // Jacobian for Ground
            case "Gnd":

                j = new ArrayList[3];
                for (int m = 0; m < 3; m++){
                    j[m] = new ArrayList<Double>();
                }

                // Loop over the jacobian and add the correct values
                for (int i = n; i < n+3; i++){
                    for(int k = 0; k < dof; k++){
                        if(k == i){
                            j[i].add(cols1.get(i), 1.0);
                        }else{
                            j[i].add(k, 0.0);
                        }
                    }
                }

                for (int p = 0; p < j.length; p++){
                    jac.add(n, j[p]);
                    n++;
                }

                return jac;

            // Jacobian for PinPin
            case "PinPin":
                j = new ArrayList[2];
                for (int m = 0; m < 2; m++){
                    j[m] = new ArrayList<Double>();
                }

                tP = link1.LinkVector(pin1)[1];
                tQ = link2.LinkVector(pin2)[1];

                // Loop over the correct columns and add values to the Jacobian matrix
                for (int i = 0; i < 2; i++){
                    for (int k = 0; k < dof; k++){
                        if(k == cols1.get(c1) && c1 < 2){
                            j[i].add(k, 1.0);
                            c1++;
                        }else if(k == cols2.get(c2) && c2 < 2){
                            j[i].add(k, -1.0);
                            c2++;
                        }else if(k == cols1.get(c1) && c1 == 2){
                            j[i].add(k, tP[i]);
                            c1++;
                        }else if(k == cols2.get(c2) && c2 == 2){
                            j[i].add(k, -tQ[i]);
                        }else{
                            j[i].add(k, 0.0);
                            j[i].add(k, 0.0);
                        }
                    }
                }

                for (int p = 0; p < j.length; p++){
                    jac.add(n, j[p]);
                    n++;
                }

                return jac;

            // Jacobian for DriveRot
            case "DriveRot":
                jacobian = new ArrayList<Double>();

                // Loop over the correct columns and add values to the Jacobian matrix
                    for (int k = 0; k < dof; k++){
                        if(k == cols1.get(2)){
                            jacobian.add(k, 1.0);
                        }else if(k == cols2.get(2)){
                            jacobian.add(k, -1.0);
                        }else{
                            jacobian.add(cols1.get(k), 0.0);
                            jacobian.add(cols2.get(k), 0.0);
                        }
                    }

                jac.add(n, jacobian);

                return jac;

            // Jacobian for PinSlot
            case "PinSlot":
                jacobian = new ArrayList<Double>();

                linkP = link1.LinkVector(pin1);
                linkQ = link2.SlotAngle(slot);

                tP = linkP[1];
                rP = linkP[2];
                rQ = linkQ[0];
                tQ = linkQ[1];

                double[] v = linkQ[2];
                double[] w = linkQ[3];

                double[] u = new double[2];
                u[0] = rP[0] - rQ[0];
                u[1] = rP[1] - rQ[1];

                double wtP = MatrixMultiply(w, tP);
                double wtQ = MatrixMultiply(w, tQ);
                double uv = MatrixMultiply(u, v);

                // Loop over the correct columns and add values to the Jacobian matrix
                    for (int k = 0; k < dof; k++){
                        if(k == cols1.get(c1) && c1 < 2){
                            jacobian.add(k, w[c1]);
                            c1++;
                        }else if(k == cols2.get(c2) && c2 < 2){
                            jacobian.add(k, -w[c2]);
                            c2++;
                        }else if(k == cols1.get(c1) && c1 == 2){
                            jacobian.add(k, wtP);
                            c1++;
                        }else if(k == cols2.get(c2) && c2 == 2){
                            jacobian.add(k, -(wtQ+uv));
                        }else{
                            jacobian.add(k, 0.0);
                            jacobian.add(k, 0.0);
                        }
                    }

                jac.add(n, jacobian);

                return jac;

            // Jacobian for Gear
            case "Gear":
                jacobian = new ArrayList<Double>();

                // Loop over the correct columns and add values to the Jacobian matrix
                for (int k = 0; k < dof; k++){
                    if(k == cols1.get(2)){
                        jacobian.add(k, rho);
                    }else if(k == cols2.get(2)){
                        jacobian.add(k, 1.0);
                    }else{
                        jacobian.add(cols1.get(k), 0.0);
                        jacobian.add(cols2.get(k), 0.0);
                    }
                }

                jac.add(n, jacobian);

                return jac;

            // Jacobian for Planetary
            case "Planetary":
                jacobian = new ArrayList<Double>();

                rP = link1.LinkVector(pin1)[2];
                rQ = link2.LinkVector(pin2)[2];

                double lambda = rhoP[0]*(offsetP[0] + link1.getTheta()) + rhoP[1]*(offsetP[1] + link2.getTheta() - Math.PI);
                w = new double[]{-Math.sin(lambda), Math.cos(lambda)};
                v = new double[]{Math.cos(lambda), Math.sin(lambda)};

                double[] rQP = new double[]{rQ[0] - rP[0], rQ[1]-rP[1]};
                double vrQP = MatrixMultiply(v, rQP);

                // Loop over the correct columns and add values to the Jacobian matrix
                for (int k = 0; k < dof; k++){
                    if(k == cols1.get(c1) && c1 < 2){
                        jacobian.add(k, w[c1]);
                        c1++;
                    }else if(k == cols2.get(c2) && c2 < 2){
                        jacobian.add(k, -w[c2]);
                        c2++;
                    }else if(k == cols1.get(c1) && c1 == 2){
                        jacobian.add(k, -vrQP*rhoP[0]);
                        c1++;
                    }else if(k == cols2.get(c2) && c2 == 2){
                        jacobian.add(k, -vrQP*rhoP[1]);
                    }else{
                        jacobian.add(k, 0.0);
                        jacobian.add(k, 0.0);
                    }
                }

                jac.add(n, jacobian);

                return jac;

            default:
                throw new UnsupportedOperationException("Not a valid constraint");
        }
    }

    /**
     * Builds the nu vector for the entire linkage
     *
     * TODO: Handle NullPointerException
     * TODO: Handle UnsupportedOperationException
     * TODO: Figure out if I want to build constraints by using a 1-based approach or 0-based approach for accessing links and pins
     *
     * @param nu - nu vector passed into the method
     * @return - A mutated nu vector
     */
    public ArrayList<Double> makeNu(ArrayList<Double> nu){
        if(nu == null){
            throw new NullPointerException("Nu vector is null");
        }

        int n = nu.size();

        // Build the Nu vector for each constraint type
        switch (type){
            case "Gnd":
                for(int i = n; i < n+3; i++){
                    nu.add(i, 0.0);
                }

                return nu;
            case "PinPin":
                for(int i = n; i < n+2; i++){
                    nu.add(i, 0.0);
                }

                return nu;
            case "DriveRot":
                nu.add(n, omega);
                return nu;
            case "PinSlot":
            case "Gear":
            case "Planetary":
                nu.add(n, 0.0);
                return nu;
            default:
                throw new UnsupportedOperationException("Not a valid constraint");
        }
    }

    /**
     * Builds the gamma vector for the entire linkage
     *
     * TODO: Handle UnsupportedOperationException
     * TODO: Handle NullPointer Exception
     *
     * @param gamma - gamma vector to be mutated
     * @param qdot - linkage vectoring containing all the link velocities
     * @param linkage - linkage we are analyzing
     * @return - a mutated gamma vector
     */
    public ArrayList<Double> makeGamma(ArrayList<Double> gamma, double[] qdot, Linkage linkage){
        if(gamma == null){
            throw new NullPointerException("Gamma vector undefined");
        }

        if(linkage == null){
            throw new NullPointerException("Linkage is undefined");
        }

        Link linkP;
        Link linkQ;

        double[] v;
        double[] rdot1;
        double[] rdot2;
        double[] rdotDiff;

        double tdot1;
        double tdot2;
        double newGammaVal;

        int n = gamma.size();

        switch (type){
            case "Gnd":
                for (int i = n; i < n+3; i++){
                    gamma.add(i, 0.0);
                }

                return gamma;

            case "PinPin":
                linkP = linkage.getLink(link1);
                linkQ = linkage.getLink(link2);

                double[] sP = linkP.LinkVector(pin1)[0];
                double[] sQ = linkQ.LinkVector(pin2)[0];

                double qdot1 = qdot[3*link1 + 2]; // For a 0-based indexing approach
                double qdot2 = qdot[3*link2 + 2]; // For a 0-based indexing approach

                for (int i = n; i < n+2; i++){
                    gamma.add(i, sP[i-n]*Math.pow(qdot1, 2) - sQ[i-n]*Math.pow(qdot2, 2));
                }

                return gamma;

            case "DriveRot":
                gamma.add(n, 0.0);
                return gamma;

            case "PinSlot":
                linkP = linkage.getLink(link1);
                linkQ = linkage.getLink(link2);

                double[] tP = linkP.LinkVector(pin1)[1];

                double[][] slotAngleQ = linkQ.SlotAngle(slot);
                v = slotAngleQ[2];
                double[] w = slotAngleQ[3];

                rdot1 = new double[]{qdot[3*link1], qdot[3*link1+1]};
                rdot2 = new double[]{qdot[3*link2], qdot[3*link2+1]};
                tdot1 = qdot[3*link1+2]; // 0-based indexing approach
                tdot2 = qdot[3*link2+2]; // 0-based indexing approach

                rdotDiff = new double[]{rdot1[0] - rdot2[0], rdot1[1] - rdot2[1]};

                double[] qP = linkP.getQ();
                double[] qQ = linkQ.getQ();
                double[] originDiff = new double[]{qP[0]-qQ[0], qP[1]-qQ[1]};

                newGammaVal = 2*tdot2*MatrixMultiply(v, rdotDiff) - MatrixMultiply(v, tP)*Math.pow(tdot1-tdot2, 2) + Math.pow(tdot2, 2)*MatrixMultiply(w, originDiff);

                gamma.add(n, newGammaVal);
                return gamma;

            case "Planetary":
                linkP = linkage.getLink(link1);
                linkQ = linkage.getLink(link2);

                double lambda = rhoP[0]*(offsetP[0] + linkP.getTheta()) + rhoP[1]*(offsetP[1] + linkQ.getTheta() - Math.PI);

                rdot1 = new double[]{qdot[3*link1], qdot[3*link1+1]};
                rdot2 = new double[]{qdot[3*link2], qdot[3*link2+1]};
                tdot1 = qdot[3*link1+2]; // 0-based indexing approach
                tdot2 = qdot[3*link2+2]; // 0-based indexing approach

                rdotDiff = new double[]{rdot1[0] - rdot2[0], rdot1[1] - rdot2[1]};

                v = new double[]{Math.cos(lambda), -Math.sin(lambda)};

                newGammaVal = -2*MatrixMultiply(v, rdotDiff)*(tdot1*rhoP[0] + tdot2*rhoP[1]);
                gamma.add(n, newGammaVal);
                return gamma;

            default:
                throw new UnsupportedOperationException("Gamma cannot be calculated for this constraint");
        }
    }

    /**
     * Performs matrix multiplication on the two provided vectors
     *
     * TODO: Handle IllegalArgumentException
     *
     * @param a - first vector in multiplication
     * @param b - second vector in multiplication
     * @return - result of Matrix multiplication
     */
    public double MatrixMultiply(double[] a, double[] b){
        double result = 0.0;
        double size = a.length;

        if(a.length != b.length){
            throw new IllegalArgumentException("The vectors do not have the correct dimensions for matrix multiplication");
        }

        for (int i = 0; i < size; i++){
            result += a[i]*b[i];
        }

        return result;
    }

}

