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
	private int head;
	private Random rngGen;
	private int[] currentReading;
	//SOUTH: +x; EAST: +y
	private final int NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3;
	private final double ploc = 0.1, pn1 = 0.05, pn2 = 0.025;

	public RobotSim(int rows, int cols, int head){
		this.rows = rows;
		this.cols = cols;
		this.head = head;
		currentReading = new int[2];
		rngGen = new Random();
		x = rngGen.nextInt(cols);
		y = rngGen.nextInt(rows);
		do{
			h = rngGen.nextInt(4);
		}while(wallAhead());
	}

	public int[] getCurrentPos() {
		int[] pos = new int[2];
		pos[0] = x;
		pos[1] = y;
		return pos;
	}

	public int[] calcCurrentReading() {
		double rng = rngGen.nextDouble();
		double addRng = 0;
		int[] reading = new int[2];
		for(int i = -2; i <= 2; i++){
			for(int j = -2; j <= 2; j++){
				if(allowedPos(x+i, y+j)){
					if(Math.abs(i) == 2 || Math.abs(j) == 2){
						addRng += pn2;
					}
					else if(Math.abs(i) == 1 || Math.abs(j) == 1){
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
		reading[0] = -1;
		reading[1] = -1;
		return reading;
	}
	
	public int[] getCurrentReading(){
		return currentReading;
	}

	private boolean allowedPos(int x, int y){
		return (x >= 0 && y >= 0 && x < rows && y < cols);
	}

	public void move() {
		updateDirection();
		makeMove();
		currentReading = calcCurrentReading();
	}

	private boolean wallAhead(){
		switch(h){
			case NORTH:
				return !allowedPos(x - 1, y);
			case EAST:
				return !allowedPos(x, y + 1);
			case SOUTH:
				return !allowedPos(x + 1, y);
			case WEST:
				return !allowedPos(x, y - 1);
			//Undefined behaviour
			default: return true;
		}
	}

	private void updateDirection(){
		if(wallAhead()){
			do{
				h = rngGen.nextInt(4);
			}while(wallAhead());
		}
		else{
			if(rngGen.nextDouble() <= 0.3){
				int hOld = h;
				do{
					h = rngGen.nextInt(4);
				}while(wallAhead() || hOld == h);
			}
		}
		return;
	}

	private void makeMove(){
		switch(h){
			case NORTH:
				x = x - 1;
				break;
			case EAST:
				y = y + 1;
				break;
			case SOUTH:
				x = x + 1;
				break;
			case WEST:
				y = y - 1;
				break;
		}
		return;
	}
}
