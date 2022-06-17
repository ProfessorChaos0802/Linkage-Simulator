package main;

import java.util.ArrayList;

public class Linkage {
    private Link[] links;
    private Constraint[] constraints;

    private int NL;
    private int NC;

    private ArrayList<Double[]> q;
    private ArrayList<Double[]> qdot;
    private ArrayList<Double[]> qddot;
    private ArrayList<Double[]> lambda;

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
    }

    public Link getLink(int link){
        return links[link];
    }

    public int numLinks(){
        return links.length;
    }


}
