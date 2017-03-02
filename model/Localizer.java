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
	private int totRealError;
	private double[][][][][][] t;
	// Not pretty using ArrayList here, but practical. Ev here is all readings.
	private ArrayList<int[]> ev;
	private ArrayList<int[]> realPos;

	public Localizer( int rows, int cols, int head) {
		this.rows = rows;
		this.cols = cols;
		this.head = head;
		totRealError = 0;

		rob = new RobotSim(rows, cols, head);

		state = new double[rows][cols][head];
		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < cols; j++) {
				for (int k = 0; k < head; k++) {
					state[i][j][k] = 1.0/(rows*cols*head);
				}
			}
		}

		Random rand = new Random();
		currentDirection = rand.nextInt(head);

		t = generateT();
		ev = new ArrayList<int[]>();
		realPos = new ArrayList<int[]>();
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
		return t[x][y][h][nX][nY][nH];
	}

	public double getOrXY( int rX, int rY, int x, int y) {
		if(rX == -1 || rY == -1){
			double prob = 1;
			for(int i = -2; i <= 2; i++){
				for(int j = -2; j <= 2; j++){
					if(allowedPos(x+i, y+j)){
						if(Math.abs(i) == 2 || Math.abs(j) == 2) prob -= pn2;
						else if(Math.abs(i) == 1 || Math.abs(j) == 1) prob -= pn1;
						else prob -= ploc;
					}
				}
			}
		return prob;
		} else {
			if(Math.abs(rX - x) > 2 || Math.abs(rY - y) > 2) return 0.0;
			else if(Math.abs(rX - x) == 2 || Math.abs(rY - y) == 2) return pn2;
			else if(Math.abs(rX - x) == 1 || Math.abs(rY - y) == 1) return pn1;
			else return ploc;
		}
	}

	private boolean allowedPos(int x, int y){
		return (x >= 0 && y >= 0 && x < rows && y < cols);
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

	public void update() {
		rob.move();
		//ev is ordered with readings in chronological order, i.e. latest is last
		ev.add(getCurrentReading());
		realPos.add(getCurrentTruePosition());
		ArrayList<double[][][]> sv = forwardBackward();
		//sv is ordered with predictions in chronological order, i.e. latest is last
		int accError = 0;
		for(int i = 0; i < realPos.size(); i++){
			state = sv.get(i);
			int error = manhattanDistance(realPos.get((i)));
			accError += error;
			if(i == realPos.size()-1) System.out.println("Latest error: " + error);
		}
		System.out.println("Round: " + ev.size());
		System.out.println("Average error (retroactively): " + (double)accError/realPos.size());
		state = sv.get(sv.size()- 1);
		totRealError += manhattanDistance(getCurrentTruePosition());
		System.out.println("Average error (relevant): " + (double)totRealError/realPos.size());
		// This system print makes it easy to import values into a matlab/python plot
		//System.out.print(error + " ");
	}

	private int manhattanDistance(int[] truePos) {
		int[] estPos = new int[2]; 
		double highestProb = 0;
		for (int i = 0; i < cols; i++) {
			for (int j = 0; j < rows; j++) {
				double currProb = getCurrentProb(i, j);
				if (currProb > highestProb) {
					highestProb = currProb;
					estPos[0] = i;
					estPos[1] = j;
				}
			}
		}
		return (Math.abs(truePos[0]-estPos[0]) + Math.abs(truePos[1]-estPos[1]));
	}

	private ArrayList<double[][][]> forwardBackward(){
		ArrayList<double[][][]> sv = new ArrayList<double[][][]>();
		ArrayList<double[][][]> fv = new ArrayList<double[][][]>();

		//The prior distribution of the initial state
		double[][][] prior = new double[rows][cols][head];
		for(int x = 0; x < rows; x++){
			for(int y = 0; y < cols; y++){
				for(int h = 0; h < head; h++){
					prior[x][y][h] = 1.0/(cols*rows*head);
				}
			}
		}
		fv.add(prior);

		for(int i = 0; i < ev.size(); i++){
			int[] currentE = ev.get(i);
			double[][] o = generateO(currentE[0], currentE[1]);
			double[][][] newF = forward(fv.get(i), o);
			fv.add(newF);
		}

		//Initial values in b should be set to one according to algorithm
		double[][][] b = new double[rows][cols][head];
		for(int x = 0; x < rows; x++){
			for(int y = 0; y < cols; y++){
				for(int h = 0; h < head; h++){
					b[x][y][h] = 1;
				}
			}
		}

		for(int i = ev.size()-1; i >= 0; i--){
			double[][][] newS = multStates(fv.get(i+1), b);
			newS = normalize(newS);
			sv.add(0, newS);
			int[] currentE = ev.get(i);
			double[][] o = generateO(currentE[0], currentE[1]);
			b = backward(b, o);
		}
		return sv;
	}

	//Should (fingers crossed) implement 15.12
	private double[][][] forward(double[][][] oldF, double[][] o){
		double[][][] newF = new double[rows][cols][head];
		for(int x = 0; x < rows; x++){
			for(int y = 0; y < cols; y++){
				for(int h = 0; h < head; h++){
					newF[x][y][h] = 0;
					// From here we calculate the probability of the state [x,y,h]
					for(int nX = 0; nX < rows; nX++){
						for(int nY = 0; nY < cols; nY++){
							for(int nH = 0; nH < head; nH++){
								// T[start:"row"][end:"column"], O(reading| [x,y]), f = prob(at (x,y,h))
								//Here on out we do the matrix computation
								newF[x][y][h] += o[x][y]*t[nX][nY][nH][x][y][h]*oldF[nX][nY][nH];
							}
						}
					}
				}
			}
		}
		//The alpha in the algorithm is for normalization (I think)
		newF = normalize(newF);
		return newF;
	}

	//Should implement 15.13
	private double[][][] backward(double[][][] oldB, double[][] o){
		double[][][] newB = new double[rows][cols][head];
		for(int x = 0; x < rows; x++){
			for(int y = 0; y < cols; y++){
				for(int h = 0; h < head; h++){
					newB[x][y][h] = 0;
					//Sum up all probability terms for state [x,y,h]
					for(int nX = 0; nX < rows; nX++){
						for(int nY = 0; nY < cols; nY++){
							for(int nH = 0; nH < head; nH++){
								newB[x][y][h] += t[x][y][h][nX][nY][nH]*o[x][y]*oldB[nX][nY][nH];
							}
						}
					}
				}
			}
		}
		return newB;
	}

	//Normalizes entries in a probability data structure
	private double[][][] normalize(double[][][] a){
		double totProb = 0.0;
		for(int x = 0; x < rows; x++){
			for(int y = 0; y < cols; y++){
				for(int h = 0; h < head; h++){
					totProb += a[x][y][h];
				}
			}
		}
		for(int x = 0; x < rows; x++){
			for(int y = 0; y < cols; y++){
				for(int h = 0; h < head; h++){
					a[x][y][h] /= totProb;
				}
			}
		}
		return a;
	}

	//Elementwise multiplication
	private double[][][] multStates(double[][][] a, double[][][] b){
		double[][][] ret = new double[rows][cols][head];
		for(int x = 0; x < rows; x++){
			for(int y = 0; y < cols; y++){
				for(int h = 0; h < head; h++){
					ret[x][y][h] = a[x][y][h]*b[x][y][h];
				}
			}
		}
		return ret;
	}

	//Looks horrendous and is not a matrix, hopefully works. Gets generated once.
	// The transition matrix for the problem. Format: T["oldPos"]["newPos"]
	private double[][][][][][] generateT(){
		double[][][][][][] t = new double[rows][cols][head][rows][cols][head];
		for(int x = 0; x < rows; x++){
			for(int y = 0; y < cols; y++){
				for(int h = 0; h < head; h++){
					for(int nX = 0; nX < rows; nX++){
						for(int nY = 0; nY < cols; nY++){
							for(int nH = 0; nH < head; nH++){
								t[x][y][h][nX][nY][nH] = calcTProb(x, y, h, nX, nY, nH);
							}
						}
					}
				}
			}
		}
		return t;
	}
	
	private double calcTProb(int x, int y, int h, int nX, int nY, int nH){
		//Must move
		if(x == nX && y == nY) return 0.0;

		// Entered a wall
		if (!allowedPos(nX,nY)) return 0.0;

		// Checks if the new direction correctly correlates to new coords
		int deltaX = 0, deltaY = 0;
		switch (nH) {
			case (NORTH) : 
				deltaX = -1;
				break;
			case (EAST) : 
				deltaY = 1;
				break;
			case (SOUTH) : 
				deltaX = 1;
				break;
			case (WEST) : 
				deltaY = -1;
				break;
		}
		if(x + deltaX != nX || y + deltaY != nY) return 0.0;
		
		//It is now established that nX,nY,nH is a possible state
		if(h == nH){
			return 0.7;
		} else {
			if(!nextIsWall(x,y,h)) return 0.3/(head-1-countWalls(x,y));
			else{
				return 1.0/(head-countWalls(x,y));
			}
		}
	}
	
	private int countWalls(int x, int y){
		int walls = 0;
		if(!allowedPos(x+1,y)) walls++;
		if(!allowedPos(x-1,y)) walls++;
		if(!allowedPos(x,y+1)) walls++;
		if(!allowedPos(x,y-1)) walls++;
		return walls;
	}
	
	private boolean nextIsWall(int x, int y, int h) {
		int deltaX = 0, deltaY = 0;
		switch (h) {
			case (NORTH) : 
				deltaX = -1;
				break;
			case (EAST) : 
				deltaY = 1;
				break;
			case (SOUTH) : 
				deltaX = 1;
				break;
			case (WEST) : 
				deltaY = -1;
				break;
		}
		return (x+deltaX < 0 || x+deltaX >= rows || y+deltaY < 0 || y+deltaY >= cols);
	}
	
	//o contains the probabilities of states based on a reading (stored in ev).
	private double[][] generateO(int rX, int rY){
		double[][] o = new double[rows][cols];
		for(int i = 0; i < rows; i++){
			for(int j = 0; j < cols; j++){
				// Can be interpreted as P(reading [rX, rY] | [x,y])
				o[i][j] = getOrXY(rX, rY, i, j)/head;
			}
		}
		return o;
	}
	
	//Only used for debugging
	private void printData(double[][][] a){
		String s = "";
		for(int i = 0; i < rows; i++){
			for(int j = 0; j < cols; j++){
				s += a[i][j][0]+a[i][j][1]+a[i][j][2]+a[i][j][3];
				s += " ";
			}
			s+= "\n";
		}
			s += "\n";
			System.out.print(s);
	}
}
