package main;

import org.junit.Test;
import org.junit.jupiter.api.DisplayName;

import java.util.ArrayList;
import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

public class LinkTest {

    @Test
    @DisplayName("Link creation for Lab 1")
    void Lab1(){
        int NL = 3;
        double ax = 5.0;
        double ay = 2.0;
        double s = 2.0;
        double l = 3.0;

        Link[] links = new Link[NL];

        int[][] colors = new int[3][3];

        colors[0] = new int[]{255, 200, 139};
        colors[1] = new int[]{145, 203, 250};
        colors[2] = new int[]{205, 208, 105};

        double theta_rect = 0.0;
        double theta_tri = 0.0;
        double theta_bar = 30 * (Math.PI / 180);

        double[] x0_rect = new double[]{5.0, 4.0};
        double[] x0_tri = new double[]{2.0, 8.0};
        double[] x0_bar = new double[]{10.0, 10.0};

        int NB_rect = 4;
        int NB_tri = 3;
        int NB_bar = 2;


        ArrayList<Double>[] pins_rect = new ArrayList[2];
        ArrayList<Double>[] pins_tri = new ArrayList[2];
        ArrayList<Double>[] pins_bar = new ArrayList[2];

        pins_rect[0] = new ArrayList<Double>();
        pins_rect[1] = new ArrayList<Double>();

        pins_tri[0] = new ArrayList<Double>();
        pins_tri[1] = new ArrayList<Double>();

        pins_bar[0] = new ArrayList<Double>();
        pins_bar[1] = new ArrayList<Double>();

        pins_rect[0].add(-ax/2);
        pins_rect[0].add(ax/2);
        pins_rect[0].add(ax/2);
        pins_rect[0].add(-ax/2);

        pins_rect[1].add(0.0);
        pins_rect[1].add(0.0);
        pins_rect[1].add(ay);
        pins_rect[1].add(ay);

        pins_tri[0].add(-s/2);
        pins_tri[0].add(s/2);
        pins_tri[0].add(0.0);

        pins_tri[1].add(-s/(2*Math.sqrt(3.0)));
        pins_tri[1].add(-s/(2*Math.sqrt(3.0)));
        pins_tri[1].add(((s/2)*Math.sqrt(3.0)) - (s / (2*Math.sqrt(3.0))));

        pins_bar[0].add(-l/2);
        pins_bar[0].add(l/2);

        pins_bar[1].add(0.0);
        pins_bar[1].add(0.0);

        links[0] = new Link(x0_rect, theta_rect, pins_rect, NB_rect, colors[0]);
        links[1] = new Link(x0_tri, theta_tri, pins_tri, NB_tri, colors[1]);
        links[2] = new Link(x0_bar, theta_bar, pins_bar, NB_bar, colors[2]);

        // Test Link Origins
        assertEquals(Arrays.toString(x0_rect), Arrays.toString(links[0].getOrigin()));
        assertEquals(Arrays.toString(x0_tri), Arrays.toString(links[1].getOrigin()));
        assertEquals(Arrays.toString(x0_bar), Arrays.toString(links[2].getOrigin()));

        // Test Link Angles
        assertEquals(theta_rect, links[0].getTheta());
        assertEquals(theta_tri, links[1].getTheta());
        assertEquals(theta_bar, links[2].getTheta());

        // Test Link Pins
        for(int i = 0; i < pins_rect.length; i++){
            assertEquals(Arrays.toString(new double[]{pins_rect[0].get(i), pins_rect[1].get(i)}), Arrays.toString(links[0].getPin(i)));
        }

        for(int i = 0; i < pins_tri.length; i++){
            assertEquals(Arrays.toString(new double[]{pins_tri[0].get(i), pins_tri[1].get(i)}), Arrays.toString(links[1].getPin(i)));
        }

        for(int i = 0; i < pins_bar.length; i++){
            assertEquals(Arrays.toString(new double[]{pins_bar[0].get(i), pins_bar[1].get(i)}), Arrays.toString(links[2].getPin(i)));
        }

        // Test Link Colors
        assertEquals(Arrays.toString(colors[0]), Arrays.toString(links[0].getColors()));
        assertEquals(Arrays.toString(colors[1]), Arrays.toString(links[1].getColors()));
        assertEquals(Arrays.toString(colors[2]), Arrays.toString(links[2].getColors()));
    }

    @Test
    @DisplayName("Link creation for Lab 2")
    void Lab2(){

    }
}