package model;

import java.awt.*;
import java.util.*;

import control.EstimatorInterface;

public class Localizer implements EstimatorInterface {
		
	private int rows, cols, head;
	private final double ploc = 0.1, pn1 = 0.05, pn2 = 0.025;
	private final int NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3;
	private double[][][] state;
	private int currentDirection;

	public Localizer( int rows, int cols, int head) {
		this.rows = rows;
		this.cols = cols;
		this.head = head;
		
		state = new double[rows][cols][head];
		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < cols; j++) {
				for (int k = 0; k < head; k++) {
					state[i][j][k] = 1.0/(rows+cols+head);
				}
			}
		}

		Random rand = new Random();
		currentDirection = rand.nextInt(4);
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
	
	/*
	 * returns the probability entry (Tij) of the transition matrix T to go from pose 
	 * i = (x, y, h) to pose j = (nX, nY, nH)
	 */	
	public double getTProb( int x, int y, int h, int nX, int nY, int nH) {
		// Entered a wall
		if (nX < 0 || nX >= cols || nY < 0 || nY >= rows) return 0.0;

		// Cannot move diagonally
		if (nX - x != 0 && nY - y != 0) return 0.0;

		// Cannot move more than one step
		if (Math.abs(nX-x) > 1 || Math.abs(nY-y) > 1) return 0.0;

		if (h == nH) {
			if (nextIsWall(nX,nY,h)) return 0.0;
			else return 0.7;
		} else {
			if (!nextIsWall(nX,nY,h)) return 0.3;
			else {
				int walls = 0;
				for (int i = 0; i < head; i++) {
					if (nextIsWall(nX,nY,i)) walls++;
				}
				return 1.0/(head-walls);
			}
		}
	}

	private boolean nextIsWall(int x, int y, int h) {
		int deltaX = 0, deltaY = 0;
		switch (h) {
			case (NORTH) : deltaY = 1;
			case (EAST) : deltaX = 1;
			case (SOUTH) : deltaY = -1;
			case (WEST) : deltaX = -1;
		}

		if (x+deltaX < 0 || x+deltaX >= cols || y+deltaX < 0 || y+deltaY >= rows) {
			return true;
		} else {
			return false;
		}
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

	}
	
	//forward-backward (page 576)	
}