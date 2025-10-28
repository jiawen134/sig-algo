/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * $Id: algorithms/Stage1.java 2014-10-18 buixuan.
 * ******************************************************/
package algorithms;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IRadarResult.Types;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Comparator;
import java.util.Random;

public class MagicMain extends Brain {
	
	/*************************   IMPORTANT   ******************************/
	/** Modifier cette variable en fonction du côté qui nous est attribué **/
	  	// - à gauche => true
		// - à droite => false
	  private boolean isLeftTeam = false;

	/*********************************************************************/
	
  //---PARAMETERS---//
  private static final double ANGLEPRECISION = 0.001;
  private static final double ANGLEPRECISIONBIS = 0.01;
  private static final double DISTANCEPRECISION = 10.;
  private static final double SAMELINEPRECISION = 125.;

  private static final double MAIN = 0xAAAAFFFF;
  private static final double SECONDARY = 0xFFFFAAAA;

  private static final int ROCKY = 0x1EBDDB;
  private static final int MARIO = 0x5ECD;
  private static final int ALPHA = 0x1EADDA;
  private static final int BETA = 0x5EC0;
  private static final int GAMMA = 0x333;
  private static final int TEAM = 0xBADDAD;
  private static final int UNDEFINED = 0xBADC0DE0;

  private static final int FIRE = 0xB52;
  private static final int POSITION = 0x7E57;
  private static final int OVER = 0xC00010FF;

  private static final int MOVETASK = 1;
  private static final int STANDINGFIRINGTASK = 2;
  private static final int BACKWARDFIRINGTASK = 3;
  private static final int MOVEBACKTASK = 5;
  private static final int TURNLEFTTASK = 6;
  private static final int TURNRIGHTTASK = 7;
  private static final int STARTINGTASK = 8;
  private static final int HUNTINGTASK = 9;
  private static final int TURNNORTHTASK = 10;
  private static final int TURNSOUTHTASK = 11;
  private static final int TURNEASTTASK = 12;
  private static final int TURNWESTTASK = 13;
  
  private static final int SINK = 0xBADC0DE1;


  //---VARIABLES---//
  private int state;
  private int fireStep;
  private double myX,myY;
  private boolean isMoving;
  private int whoAmI;
/*  private int fireRythm,rythm,counter;
  private int countDown;*/
  private double targetX,targetY;
  private boolean fireOrder;
  private Random rand;
  private HashMap<Integer, ArrayList<Double>> allies;
  private ArrayList<ArrayList<Double>> targets;
	private int stepNumberLastFire;
	private int stepNumber;
	private int stepNumberMoveBack;
	private double endTaskDirection;
	private String huntingMode;
	private boolean findNewShotAngle;
	private boolean isMovingBack;
	private int counter;
	

  //---CONSTRUCTORS---//
  public MagicMain() {
    super();
    allies = new HashMap<Integer, ArrayList<Double>>();
    ArrayList<Double> temp = new ArrayList<Double>(3);
    temp.add(0.);
    temp.add(0.);
    temp.add(0.);
    allies.put(ALPHA, temp);
    allies.put(BETA, temp);
    allies.put(GAMMA, temp);
    allies.put(ROCKY, temp);
    allies.put(MARIO, temp);
    targets = new ArrayList<ArrayList<Double>>(5);
    rand = new Random();
  }

  //---ABSTRACT-METHODS-IMPLEMENTATION---//
  public void activate() {
    //ODOMETRY CODE
    whoAmI = GAMMA;
    for (IRadarResult o: detectRadar())
      if (isSameDirection(o.getObjectDirection(),Parameters.NORTH)) whoAmI=ALPHA;
    for (IRadarResult o: detectRadar())
      if (isSameDirection(o.getObjectDirection(),Parameters.SOUTH) && whoAmI!=GAMMA) whoAmI=BETA;

    if (isLeftTeam) {
    		//KD RUNNERS
	    if (whoAmI == GAMMA){
	        myX=Parameters.teamAMainBot1InitX;
	        myY=Parameters.teamAMainBot1InitY;
	      } else {
	        myX=Parameters.teamAMainBot2InitX;
	        myY=Parameters.teamAMainBot2InitY;
	      }
	      if (whoAmI == ALPHA){
	        myX=Parameters.teamAMainBot3InitX;
	        myY=Parameters.teamAMainBot3InitY;
	      }
    }else {
    		//FANTOM DANGER
	    if (whoAmI == GAMMA){
	      myX=Parameters.teamBMainBot1InitX;
	      myY=Parameters.teamBMainBot1InitY;
	    } else {
	      myX=Parameters.teamBMainBot2InitX;
	      myY=Parameters.teamBMainBot2InitY;
	    }
	    if (whoAmI == ALPHA){
	      myX=Parameters.teamBMainBot3InitX;
	      myY=Parameters.teamBMainBot3InitY;
	    }
    }

    //INIT
    //state=MOVETASK;
    state=STARTINGTASK;
    isMoving=false;
    fireOrder=false;
    //fireRythm=0;
    targetX=0;
    targetY=0;
	stepNumberLastFire=0;
	stepNumber=0;
	stepNumberMoveBack=0;
	findNewShotAngle=false;
	isMovingBack=false;
	counter=0;
  }
  public void step() {
	stepNumber++;
	if (stepNumber > 3000 && state == STARTINGTASK) {
		state = MOVETASK;
	} 
	
	if(counter>460) {
		counter=0;
	}

	if (getHealth()==0) state=SINK;
	
    //ODOMETRY CODE
    if (isMoving){
      myX+=Parameters.teamAMainBotSpeed*Math.cos(myGetHeading());
      myY+=Parameters.teamAMainBotSpeed*Math.sin(myGetHeading());
      realCoords();
      isMoving=false;
    }
    
    if (isMovingBack) {
    		myX-=Parameters.teamAMainBotSpeed*Math.cos(myGetHeading());
        myY-=Parameters.teamAMainBotSpeed*Math.sin(myGetHeading());
        realCoords();
        isMovingBack=false;
    }
    //DEBUG MESSAGE
    boolean debug=true;
    if (debug && whoAmI == ALPHA && state!=SINK) {
      sendLogMessage("#ALPHA *thinks* (x,y)= ("+(int)myX+", "+(int)myY+") theta= "+(int)(myGetHeading()*180/(double)Math.PI)+"°. #State= "+state);
    }
    if (debug && whoAmI == BETA && state!=SINK) {
      sendLogMessage("#BETA *thinks* (x,y)= ("+(int)myX+", "+(int)myY+") theta= "+(int)(myGetHeading()*180/(double)Math.PI)+"°. #State= "+state);
    }
    if (debug && whoAmI == GAMMA && state!=SINK) {
      sendLogMessage("#GAMMA *thinks* (x,y)= ("+(int)myX+", "+(int)myY+") theta= "+(int)(myGetHeading()*180/(double)Math.PI)+"°. #State= "+state);
    }
    if (debug && fireOrder) {
    		counter++;
    		sendLogMessage("Firing enemy!!");
    }

    //COMMUNICATION
    targets.clear(); // prepare for the next targets set
    ArrayList<String> messages=fetchAllMessages();
    for (String m: messages) if (Integer.parseInt(m.split(":")[1])==whoAmI || Integer.parseInt(m.split(":")[1])==TEAM) process(m);

    broadcast(whoAmI+":"+TEAM+":"+POSITION+":"+myX+":"+myY+":"+getHeading()+":"+OVER);

    //RADAR DETECTION
    for (IRadarResult o: detectRadar()){
      if (o.getObjectType()== Types.OpponentMainBot || o.getObjectType()== Types.OpponentSecondaryBot) {
        double enemyX=myX+o.getObjectDistance()*Math.cos(o.getObjectDirection());
        double enemyY=myY+o.getObjectDistance()*Math.sin(o.getObjectDirection());
        broadcast(whoAmI+":"+TEAM+":"+FIRE+":"+(o.getObjectType()== Types.OpponentMainBot?MAIN:SECONDARY)+":"+enemyX+":"+enemyY+":"+OVER);
      }
      if (o.getObjectDistance() < 120 && o.getObjectType() != Types.BULLET) {
			if (state==MOVETASK) {
				//System.out.println("detected something, must move back");
				state=MOVEBACKTASK;
				stepNumberMoveBack = stepNumber;
				return;
			}
		}
    }
    if (fireOrder && !findNewShotAngle) {
      setTarget();
    }

    //AUTOMATON
    
    if (myX<=Parameters.teamAMainBotRadius) {
    		if (isHeading(Parameters.EAST)) {
			state=MOVETASK;
			return;
		}
		state=TURNEASTTASK;
		return;
	}
	if (myX>=(3000-Parameters.teamAMainBotRadius)) {
		if (isHeading(Parameters.WEST)) {
			state=MOVETASK;
			return;
		}
		state=TURNWESTTASK;
		return;
	}
	if (myY<=Parameters.teamAMainBotRadius) {
		if (isHeading(Parameters.SOUTH)) {
			state=MOVETASK;
			return;
		}
		state=TURNSOUTHTASK;
		return;
	}
	if (myY>=(2000-Parameters.teamAMainBotRadius)) {
		state=TURNNORTHTASK;
		if (isHeading(Parameters.NORTH)) {
			state=MOVETASK;
			return;
		}
		return;
	}
    
    if (state == STARTINGTASK) {
	    	if (stepNumber > 100 && canFireLatency()) {
	    		if (fireOrder /*&& stepNumber>1000*/) {
	    			if (canIShot(targetX, targetY)) {
	    				firePosition(targetX, targetY);
		    			stepNumberLastFire = stepNumber;
		    			return;
				}
	    		}
			fire(myGetHeading());
			stepNumberLastFire = stepNumber;
			return;
		}
	    	myMove();
    }
    
    if (fireOrder && canFireLatency()) {
	    	if (canIShot(targetX, targetY)) {
    			firePosition(targetX,targetY);
    		    stepNumberLastFire = stepNumber;
    		    return;
	    	} 
    }
    
    if (!fireOrder && stepNumber>6000 && targets.size() > 0 && state != HUNTINGTASK) {
		//System.out.println("hunting ?");
    		double tx = targets.get(0).get(1);
		double ty = targets.get(0).get(2);
		double distX = Math.abs(tx-myX);
		double distY = Math.abs(ty-myY);
		if (distX > distY || distY < 200) { // se rapprocher sur le plan x
			state = HUNTINGTASK;
			huntingMode = "x";
			//System.out.println("yes with x");
			return;
		} else if (distX > 200){
			state = HUNTINGTASK;
			huntingMode = "y";
			//System.out.println("yes with y");
			return;
		}
    }
    
    if (state == HUNTINGTASK && !fireOrder) {
    		if (targets.isEmpty()) {
    			state = MOVETASK;
    			huntingMode = "";
    			return;
    		}
    		
    		if (huntingMode.equals("x")) {
    			double tx = targets.get(0).get(1);
    			double distX = Math.abs(tx-myX);
    			if (distX<200) {
    				huntingMode = "";
    				state = MOVETASK;
    				return;
    			} else {
    				if (tx < myX) { // target à gauche
    					if (isHeading(Parameters.WEST)) {
    						myMove();
    						return;
    					} else {
    						if (myGetHeading() < Math.PI && myGetHeading() > 0) {
    					        stepTurn(Parameters.Direction.RIGHT);
    					    }
					    else {
					    		stepTurn(Parameters.Direction.LEFT);
					    }
    						return;
    					}
    				} else { //target à gauche
    					if (isHeading(Parameters.EAST)) {
    						myMove();
    						return;
    					} else {
    						if (myGetHeading() < Math.PI && myGetHeading() > 0) {
    					        stepTurn(Parameters.Direction.RIGHT);
    					    }
					    else {
					    		stepTurn(Parameters.Direction.LEFT);
					    }
    						return;
    					}
    				}
    			}
    		} else {
    			double ty = targets.get(0).get(2);
    			double distY = Math.abs(ty-myY);
    			if (distY<200) {
    				huntingMode = "";
    				state = MOVETASK;
    				return;
    			} else {
    				if (ty < myY) { // target en haut
    					if (isHeading(Parameters.NORTH)) {
    						myMove();
    						return;
    					} else {
    						if (myGetHeading() < Math.PI / 2 || myGetHeading() > 3 * Math.PI / 2) {
    					        stepTurn(Parameters.Direction.LEFT);
    					    } else {
    					        stepTurn(Parameters.Direction.RIGHT);
    					    }
    					    return;
    					}
    				} else { //target en bas
    					if (isHeading(Parameters.SOUTH)) {
    						myMove();
    						return;
    					} else {
    						if (myGetHeading() < Math.PI / 2 || myGetHeading() > 3 * Math.PI / 2) {
    					        stepTurn(Parameters.Direction.LEFT);
    					    } else {
    					        stepTurn(Parameters.Direction.RIGHT);
    					    }
    					    return;
    					}
    				}
    			}
    		}
    }
    
    if (state == HUNTINGTASK && fireOrder) {
    		state = MOVETASK;
    		myMove();
    		huntingMode = "";
    		return;
    }


    if (state==MOVETASK && detectFront().getObjectType()!=IFrontSensorResult.Types.WALL) {
	    	if (canFireLatency()) {
	        for(int i = 0; i < 10; i++) {
	          double angle=rand.nextDouble() * Math.PI / 6 - Math.PI / 12;
	          double x = myX + Parameters.bulletRange*Math.cos(myGetHeading()+angle);
	          double y = myY + Parameters.bulletRange*Math.sin(myGetHeading()+angle);
	          if (canIShot(x,y)) {
	            firePosition(x,y);
	            stepNumberLastFire = stepNumber;
	            return;
	          }
	        }
	      }
	     
	      myMove();
	      return;
    }
    
    if (state==MOVETASK && detectFront().getObjectType()==IFrontSensorResult.Types.WALL) {
			if ((myX>2915 && myY>1915) || (myX > 2915 && myY<85) || (myX<85 && myY<85) || (myX<85 && myY>1915)) {
				state=TURNLEFTTASK;
    		  		endTaskDirection = getHeading() + Parameters.LEFTTURNFULLANGLE;
    		  		stepTurn(Parameters.Direction.LEFT);
    		  		return;
			} else if (myX>2915 || myX<85) {
				if (isHeading(Parameters.EAST) || isHeading(Parameters.WEST)){
					state=TURNLEFTTASK;
		  			endTaskDirection = getHeading() + Parameters.LEFTTURNFULLANGLE;
		  			stepTurn(Parameters.Direction.LEFT);
		  			return;
				} else {
					myMove();
					return;
				}
			} else if (myY>1915 || myY<85){
				if (isHeading(Parameters.NORTH) || isHeading(Parameters.SOUTH)){
					state=TURNLEFTTASK;
		  			endTaskDirection = getHeading() + Parameters.LEFTTURNFULLANGLE;
		  			stepTurn(Parameters.Direction.LEFT);
		  			return;
				} else {
					myMove();
					return;
				}
			} else {
				myMove();
				return;
			}
    			
    	
    }
    		

    if (state==STANDINGFIRINGTASK) {
      state=MOVETASK;
      if ((++fireStep % 2) == 0 && counter < 415) {
        moveBack();
        myX-=Parameters.teamAMainBotSpeed*Math.cos(getHeading());
        myY-=Parameters.teamAMainBotSpeed*Math.sin(getHeading());
        realCoords();
      }
      else {
        myMove();
      }
      return;
    }
    if (state==BACKWARDFIRINGTASK) {
      state=MOVETASK;
      if ((++fireStep % 1) == 0) {
        moveBack();
        myX-=Parameters.teamAMainBotSpeed*Math.cos(getHeading());
        myY-=Parameters.teamAMainBotSpeed*Math.sin(getHeading());
        realCoords();
      }
      else {
        myMove();
      }
      return;
    }
    

	if (state==MOVEBACKTASK) {
		if (stepNumber < stepNumberMoveBack + 25) {
			myMoveBack();
			return;
		} else {
			if (Math.random() < 0.5) {	    	 		
    	 		state=TURNLEFTTASK;
    	 		endTaskDirection = getHeading() + Parameters.LEFTTURNFULLANGLE;
    	 		stepTurn(Parameters.Direction.LEFT);
	    	 	} else {
	    	 		state=TURNRIGHTTASK;
	    	 		endTaskDirection = getHeading() + Parameters.RIGHTTURNFULLANGLE;
	    	 		stepTurn(Parameters.Direction.RIGHT);
	    	 	}
			return;
		} 
	}
	
	if (state==TURNRIGHTTASK) {
		if (isHeading(endTaskDirection)) {
			state=MOVETASK;
			myMove();
		} else {
			stepTurn(Parameters.Direction.RIGHT);
		}
		return;
	}
    
    if (state==TURNLEFTTASK) {
		if (isHeading(endTaskDirection)) {
			state=MOVETASK;
			myMove();
		} else {
			stepTurn(Parameters.Direction.LEFT);
		}
		return;
	}
    
    /* 4 directions turning */
    if (state==TURNNORTHTASK && !(isHeading(Parameters.NORTH))) {
      if (myGetHeading() < Math.PI / 2 || myGetHeading() > 3 * Math.PI / 2) {
        stepTurn(Parameters.Direction.LEFT);
      }
      else {
        stepTurn(Parameters.Direction.RIGHT);
      }
      return;
    }
    if (state==TURNNORTHTASK && isHeading(Parameters.NORTH)) {
      state=MOVETASK;
      myMove();
      return;
    }
    if (state==TURNSOUTHTASK && !(isHeading(Parameters.SOUTH))) {
      if (myGetHeading() < Math.PI / 2 || myGetHeading() > 3 * Math.PI / 2) {
        stepTurn(Parameters.Direction.RIGHT);
      }
      else {
        stepTurn(Parameters.Direction.LEFT);
      }
      return;
    }
    if (state==TURNSOUTHTASK && isHeading(Parameters.SOUTH)) {
      state=MOVETASK;
      myMove();
      return;
    }
    if (state==TURNEASTTASK && !(isHeading(Parameters.EAST))) {
      if (myGetHeading() < Math.PI && myGetHeading() > 0) {
        stepTurn(Parameters.Direction.LEFT);
      }
      else {
        stepTurn(Parameters.Direction.RIGHT);
      }
      return;
    }
    if (state==TURNEASTTASK && isHeading(Parameters.EAST)) {
      state=MOVETASK;
      myMove();
      return;
    }
    if (state==TURNWESTTASK && !(isHeading(Parameters.WEST))) {
      if (myGetHeading() < Math.PI && myGetHeading() > 0) {
        stepTurn(Parameters.Direction.RIGHT);
      }
      else {
        stepTurn(Parameters.Direction.LEFT);
      }
      return;
    }
    if (state==TURNWESTTASK && isHeading(Parameters.WEST)) {
      state=MOVETASK;
      myMove();
      return;
    }

    if (state==SINK) {
      //myMove();
      return;
    }
    if (true) {
      return;
    }
  }
  private void myMove(){
	  isMoving=true;
	  move();
  }
  private void myMoveBack() {
	  isMovingBack=true;
	  moveBack();
  }
  private double myGetHeading(){
    return normalizeRadian(getHeading());
  }
  private double normalizeRadian(double angle){
    double result = angle;
    while(result<0) result+=2*Math.PI;
    while(result>=2*Math.PI) result-=2*Math.PI;
    return result;
  }
  private boolean isSameDirection(double dir1, double dir2){
    return Math.abs(normalizeRadian(dir1)-normalizeRadian(dir2))<ANGLEPRECISION;
  }
 
  private void process(String message){
    if (Integer.parseInt(message.split(":")[2])==FIRE) {
      double x=Double.parseDouble(message.split(":")[4]);
      double y=Double.parseDouble(message.split(":")[5]);
      boolean already = false;
      for (ArrayList<Double> list : targets) {
        if ((Math.abs(x - list.get(1)) <= DISTANCEPRECISION) &&
            (Math.abs(y - list.get(2)) <= DISTANCEPRECISION)) {
          already = true;
        }
      }
      if (!already) {
        ArrayList<Double> target = new ArrayList<Double>(3);
        target.add(Double.parseDouble(message.split(":")[3]));
        target.add(x);
        target.add(y);
        targets.add(target);
      }
      fireOrder=true;
      //countDown=0;
    }
    if (Integer.parseInt(message.split(":")[2])==POSITION) {
      ArrayList<Double> temp = new ArrayList<Double>(2);
      temp.add(Double.parseDouble(message.split(":")[3]));
      temp.add(Double.parseDouble(message.split(":")[4]));
      temp.add(Double.parseDouble(message.split(":")[5]));
      allies.replace(Integer.parseInt(message.split(":")[0]), temp);
    }
  }
  private void setTarget() {
    ArrayList<ArrayList<Double>> realTargets = new ArrayList<ArrayList<Double>>(5);
    for (ArrayList<Double> target : targets) {
      if (distance(myX, myY, target.get(1), target.get(2)) <= Parameters.bulletRange) {
        realTargets.add(target);
      }
    }
    realTargets.sort(new Comparator<ArrayList<Double>>() {
      @Override
      // order: mains that have our mains inside their radar range > secondary > mains that can't detect our mains without their secondairies.
      // In each category, the closest target is the priority
      public int compare(ArrayList<Double> l1, ArrayList<Double> l2) {
        if (l1.get(0) == l2.get(0)) {
          return Double.compare(distance(myX, myY, l2.get(1), l2.get(2)), distance(myX, myY, l1.get(1), l1.get(2)));
        }
        else if (l1.get(0) == MAIN) {
          return distance(myX, myY, l1.get(1), l1.get(2)) <= Parameters.teamBMainBotFrontalDetectionRange ? 1 : -1;
        }
        else { // (l2.get(0) == MAIN) {
          return distance(myX, myY, l2.get(1), l2.get(2)) > Parameters.teamBMainBotFrontalDetectionRange ? 1 : -1;
        }
      }
    });
    for (ArrayList<Double> target : realTargets) {
      if (canIShot(target.get(1), target.get(2))) {
        targetX = target.get(1);
        targetY = target.get(2);
        if (distance(myX, myY, targetX, targetY) > Parameters.teamBSecondaryBotFrontalDetectionRange + 100) {
          state=STANDINGFIRINGTASK;
        }
        else {
          state=BACKWARDFIRINGTASK;
        }
        return;
      }
    }
    fireOrder = false;
  }
  private void firePosition(double x, double y) {
    if (myX<=x) fire(Math.atan((y-myY)/(double)(x-myX)));
    else fire(Math.PI+Math.atan((y-myY)/(double)(x-myX)));
    return;
  }

  private boolean canIShot(double x, double y) {
    // y = a*x+b is the line between the robot and the target
    double a = (y - myY) / (x - myX);
    double b = myY - a*myX;
    for (ArrayList<Double> ally : allies.values()) {
      if (distance(myX, myY, ally.get(0), ally.get(1)) <= DISTANCEPRECISION) {
        continue; // this ally is this robot
      }
      double angleToAlly = getDirectionToTarget(ally.get(0), ally.get(1));
      double angleToTarget = getDirectionToTarget(x, y);
      if (Math.abs(angleToAlly - angleToTarget) < Math.PI / 12.0
    		  && distance(myX, myY, ally.get(0), ally.get(1)) < distance(myX, myY, x, y)) {
    	  	/*System.out.println("myX and myY" + myX + " "+myY);
    	  	System.out.println("ang to ally "+angleToAlly);
    	  	System.out.println("ang to target "+ angleToTarget);
    	  	System.out.println("cannot shot cauz ally in the fire field " + ally);
    	  	*/
    	  	//System.out.println("theree");
    	  	return false;
      }
      
      if (getHeading() == Parameters.EAST) {
    	  	if (Math.abs(ally.get(1) - myY) < 15 && ally.get(0) > myX ) {
    	  		//System.out.println("hhohoohoho");
    	  		return false;
    	  	}
      }
      if (getHeading() == Parameters.WEST) {
    	  	if (Math.abs(ally.get(1) - myY) < 15 && ally.get(0) < myX ) {
    	  		return false;
    	  	}
      }
      if (getHeading() == Parameters.SOUTH){ 
      	if (Math.abs(ally.get(0) - myX) < 15 && ally.get(0) > myX ) {
    	  		return false;
    	  	}
      }
      if (getHeading() == Parameters.NORTH) {
  	  	if (Math.abs(ally.get(0) - myX) < 15 && ally.get(0) < myX ) {
    	  		return false;
    	  	}
    }
      // y = allyA*x+allyB is the line the ally is following
      double allyA = Math.tan(ally.get(2));
      double allyB = ally.get(1) - allyA * ally.get(0);
      // (allyX,allyY) is the intersection between these 2 lines
      // (where the ally and the bullet could potentially collide)
      double allyX = (b - allyB) / (allyA - a);
      double allyY = a * allyX + b;
      // this doesn't take into account the speed of the ally and the bullet
      if (distance(ally.get(0), ally.get(1), allyX, allyY) <= SAMELINEPRECISION &&
         (x >= allyX && allyX >= myX || x <= allyX && allyX <= myX) &&
         (y >= allyY && allyY >= myY || y <= allyY && allyY <= myY)) {
          return false;
      }
    }
    if (detectFront().getObjectType() == IFrontSensorResult.Types.TeamMainBot 
    		|| detectFront().getObjectType() == IFrontSensorResult.Types.TeamSecondaryBot)
    		return false;
    return true;
  }
  private double distance(double x1, double y1, double x2, double y2) {
    return Math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
  }
  private void realCoords() {
    myX = myX < 0 ? 0 : myX;
    myX = myX > 3000 ? 3000 : myX;
    myY = myY < 0 ? 0 : myY;
    myY = myY >2000 ? 2000 : myY;
  }
	private boolean canFireLatency() {
		return stepNumber > stepNumberLastFire + Parameters.bulletFiringLatency;
		
	}
	private boolean isHeading(double dir){
	    return Math.abs(Math.sin(getHeading()-dir))<ANGLEPRECISIONBIS;
	}
	private double getDirectionToTarget(double x, double y) {
	    double dir = Math.atan2(y - myY, x - myX);
	    return normalizeRadian(dir);
	 }

	
}