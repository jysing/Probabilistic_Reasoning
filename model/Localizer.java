package model;

import control.EstimatorInterface;

public class Localizer implements EstimatorInterface {
		
	private int rows, cols, head;
	private final double ploc = 0.1, pn1 = 0.05, pn2 = 0.025;
	private double[][][] state;

	public Localizer( int rows, int cols, int head) {
		this.rows = rows;
		this.cols = cols;
		this.head = head;
		
		state = new double[rows][cols][head];
	}	
	
	public int getNumRows() {
		return rows;
	}
	
	public int getNumCols() {
		return cols;
	}
	
	public int getNumHead() {
		return head;
	}
	
	public double getTProb( int x, int y, int h, int nX, int nY, int nH) {
		return 0.0;
	}

	public double getOrXY( int rX, int rY, int x, int y) {
		return 0.1;
	}


	public int[] getCurrentTruePosition() {
		
		int[] ret = new int[2];
		ret[0] = rows/2;
		ret[1] = cols/2;
		return ret;

	}

	public int[] getCurrentReading() {
		int[] ret = null;
		return ret;
	}


	public double getCurrentProb( int x, int y) {
		double ret = 0.0;
		return ret;
	}
	
	public void update() {
		System.out.println("Nothing is happening, no model to go for...");
	}
	
	
}