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
    public ArrayList<Double> MakePhi(Linkage linkage, ArrayList<Double> phi) {
        // Check that phi is valid
        if (phi == null) {
            throw new NullPointerException();
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
                throw new UnsupportedOperationException();
        }
    }

    public ArrayList<ArrayList<Double>> MakeJac(Linkage linkage, ArrayList<ArrayList<Double>> jac){
        Link link1 = linkage.getLink(this.link1);
        Link link2 = linkage.getLink(this.link2);

        ArrayList<Integer> cols1 = new ArrayList<Integer>();
        ArrayList<Integer> cols2 = new ArrayList<Integer>();

        int dof = linkage.numLinks()*3;

        for (int i = 0; i < 3; i++){
            cols1.add(i, 3*(this.link1 - 1) + i);
            cols2.add(i, 3*(this.link2 - 1) + i);
        }

        switch (type){

            // Jacobian for Ground
            case "Gnd":
                int n = jac.size();

                ArrayList<Double>[] j = new ArrayList[3];
                for (int m = 0; m < 3; m++){
                    j[m] = new ArrayList<Double>();
                }

                for (int i = n; i < n+3; i++){
                    for(int k = 0; k < dof; k++){
                        if(k % i == 0){
                            j[i].add(1.0);
                        }else{
                            j[i].add(0.0);
                        }
                    }
                }

                for (int p = 0; p < j.length; p++){
                    jac.add(j[p]);
                }

                return jac;
            default:
                throw new UnsupportedOperationException();

        }
    }

}

