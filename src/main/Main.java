package main;

import javax.swing.*;
import java.awt.*;
import java.io.IOException;

public class Main {

    public static final int DELAY = 30; // Number of milliseconds between updates in the game
    private static final int FRAME_WIDTH = 1200;
    private static final int FRAME_HEIGHT = 1025;
    private Dimension dim = new Dimension(FRAME_WIDTH, FRAME_HEIGHT);
    private Dimension selectorDim = new Dimension(60, 30);
    private JFrame homeFrame;

    /**
     * Empty constructor that gets the ball rolling
     *
     * @throws IOException
     */
    public Main() throws IOException {
        homeFrame = new JFrame();
        homeFrame.setPreferredSize(this.dim);
        homeFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        homeFrame.getContentPane().setBackground(Color.BLACK);
        homeFrame.setVisible(true);
        homeFrame.pack();
        homeFrame.repaint();
    }

    /**
     * Causes the file to run by creating an instance of Main
     *
     * @throws IOException
     */
    public static void main(String[] args) throws IOException {
        Main newGame = new Main();
    }
}