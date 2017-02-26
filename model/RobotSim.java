package model;

import java.awt.*;
import java.util.*;

import control.EstimatorInterface;

public class RobotSim {

	private int x;
	private int y;
	private int h;
	private int rows;
	private int cols;
	private Random rngGen;
	private final int NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3;
	private final double ploc = 0.1, pn1 = 0.05, pn2 = 0.025;

	public RobotSim(int rows, int cols, int x, int y, int h){
		this.rows = rows;
		this.cols = cols;
		this.x = x;
		this.y = y;
		this.h = h;
		rngGen = new Random();
	}

	public int[] getCurrentPos() {
		int pos = int[2];
		pos[0] = x;
		pos[1] = y;
		return pos;
	}

	public int[] getCurrentReading() {
		double rng = rngGen.nextDouble();
		double addRng = 0;
		int[] reading = new int[2];
		for(int i = -2; i <= 2; i++){
			for(int j = -2; j <= 2; j++){
				if(allowedPos(x+i, y+j)){
					if(abs(i) == 2 || abs(j) == 2){
						addRng += pn2;
					}
					else if(abs(i) == 1 || abs(j) == 1){
						addRng += pn1;
					}
					else{ 	//i = j = 0
						addRng += ploc;
					}
					if(addRng >= rng){
						reading[0] = x+i;
						reading[1] = y+j;
						return reading;
					}
				}
			}
		}
		//If addRng >= rng never is satisfied, then we get a "Nothing" reading
		return null;
	}

	private boolean allowedPos(int x, int y){
		return (x >= 0 && y >= 0 && x < cols && y < rows);
	}

	public void move() {
		updateDirection();
		makeMove();
		return;
	}

	private boolean wallAhead(){
		switch(h){
			case NORTH:
				return allowedPos(x, y - 1);
			case EAST:
				return allowedPos(x + 1, y);
			case SOUTH:
				return allowedPos(x, y + 1);
			case WEST:
				return allowedPos(x - 1, y);
		}
	}

	private void updateDirection(){
		if(wallAhead()){
			do{
				h = rngGen.nextInt(3);
			}while(wallAhead());
		}
		else{
			if(rngGen.nextDouble() <= 0.3){
				do{
					h = rngGen.nextInt(3);
				}while(wallAhead())
			}
		}
		return;
	}

	private void makeMove(){
		switch(h){
			case NORTH:
				y = y - 1;
				break;
			case EAST:
				x = x + 1;
				break;
			case SOUTH:
				y = y + 1;
				break;
			case WEST:
				x = x - 1;
				break;
		}
		return;
	}
}