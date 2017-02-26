package model;

import java.awt.*;
import java.util.*;

import control.EstimatorInterface;

public class Localizer implements EstimatorInterface {

	private int rows, cols, head;
	private final double ploc = 0.1, pn1 = 0.05, pn2 = 0.025;
	private final int NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3;
	private int currentDirection;
	private RobotSim rob;
	private double[][][] state;
	private double[][][][][][] t;
	// Not pretty using ArrayList here, but practical. Ev here is all readings.
	private ArrayList<int[][]> ev;

	public Localizer( int rows, int cols, int head) {
		this.rows = rows;
		this.cols = cols;
		this.head = head;

		rob = new RobotSim();

		state = new double[cols][rows][head];
		for (int i = 0; i < cols; i++) {
			for (int j = 0; j < rows; j++) {
				for (int k = 0; k < head; k++) {
					state[i][j][k] = 1.0/(rows+cols+head);
				}
			}
		}

		Random rand = new Random();
		currentDirection = rand.nextInt(4);

		t = generateT();
		ev = new ArrayList<int[][]>();
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
		//Must move
		if(x == nX && y == nY) return 0.0;

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

		return (x+deltaX < 0 || x+deltaX >= cols || y+deltaX < 0 || y+deltaY >= rows);
	}

	public double getOrXY( int rX, int rY, int x, int y) {
		if(rx == -1 || ry == -1){
			double prob = 1;
			for(int i = -2; i <= 2; i++){
				for(int j = -2; j <= 2; j++){
					if(allowedPos(x+i, y+j)){
						if(abs(i) == 2 || abs(j) == 2){
							prob-= pn2;
						}
						else if(abs(i) == 1 || abs(j) == 1){
							prob =- pn1;
						}
						else{ 	//i = j = 0
							prob -= ploc;
						}
					}
				}
			}
		return prob;
		}
		else{
			if(abs(rX - x) > 2 || abs(rY - y) > 2) return 0.0;
			else if(abs(rX - x) == 2 || abs(rY - y) == 2) return pn2;
			else if(abs(rX - x) == 1 || abs(rY - y) == 1) return pn1;
			else return ploc;
		}
	}

	private boolean allowedPos(int x, int y){
		return (x >= 0 && y >= 0 && x < cols && y < rows);
	}

	public int[] getCurrentTruePosition() {
		return rob.getCurrentPos();
	}

	public int[] getCurrentReading() {
		return rob.getCurrentReading();
	}


	public double getCurrentProb( int x, int y) {
		double prob = 0.0;
		for (int i = 0; i < head; i++) {
			prob += state[x][y][i];
		}
		return prob;
	}

	// todo
	public void update() {
		// rob.move();
		// getCurrentReading();
		// forwardBackward();
		// calcuate manhattan distance
	}

	// todo
	private void forwardBackward() {

	}

	//Should (fingers crossed) implement 15.12
	private double[][][] forward(double[][][] oldF, double[][] o){
		double[][][] newF = new double[cols][rows][head];
		double totUnscProb = 0;
		for(int x = 0; x < cols; x++){
			for(int y = 0; y < rows; x++){
				for(int h = 0; h < head; h++){
					// From here we calculate the probability of the state [x,y,h]
					double unscProb = 0.0;
					for(int nX = 0; nX < cols; nX++){
						for(int nY = 0; nY < rows; nY++){
							for(int nH = 0; nH < head; nH++){
								unscProb += oldF[x][y][h]*o[x][y]*t[x][y][h][nX][nY][nH];
							}
						}
					}
					newF[x][y][h] = unscProb;
					totUnscProb += unscProb;
				}
			}
		}

		//The "forward-message" is scaled.
		for(int x = 0; x < cols; x++){
			for(int y = 0; y < rows; y++){
				for(int h = 0; h < head; h++){
					newF[x][y][h] /= totUnscProb;
				}
			}
		}
		return newF;
	}

	//Looks horrendous and is not a matrix, hopefully works. Gets generated once.
	// The transition matrix for the problem. Format: T["oldPos"]["newPos"]
	private double[][][][][][] generateT(){
		double[][][][][][] t = new double[rows][cols][head][rows][cols][head];
		for(int x = 0; x < cols; x++){
			for(int y = 0; y < rows; y++){
				for(int h = 0; h < head; h++){
					for(int nX = 0; nX < cols; nX++){
						for(int nY = 0; nY < rows; nY++){
							for(int nH = 0; nH < head; nH++){
								t[x][y][h][nX][nY][nH] = getTProb(x, y, h, nX, nY, nH);
							}
						}
					}
				}
			}
		}
		return t;
	}

	//o contains the probabilities of states based on a reading (stored in ev).
	private double[][] generateO(int rX, int rY){
		double[][] o = new double[cols][rows];
		for(int i = 0; i < cols; i++){
			for(int j = 0; j1 < rows; j++){
				// Can be interpreted as P(reading [rX, rY] | [x,y])
				o[i][j] = getOrXY(rX, rY, i, j);
			}
		}
	}

	//forward-backward (page 576)
}
