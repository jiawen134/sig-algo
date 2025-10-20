/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * $Id: algorithms/BrainCanevas.java 2014-10-19 buixuan.
 * ******************************************************/
package algorithms;

import robotsimulator.Brain;
import characteristics.IFrontSensorResult;
import characteristics.Parameters;

public class BrainCanevas extends Brain {
  //---PARAMETERS---//
  private static final double HEADINGPRECISION = 0.01;
  private static final double POSITIONPRECISION = 20.0;

  private static final int STATE_ALIGN_NORTH = 0;
  private static final int STATE_FOLLOW_WALL_MOVE = 1;
  private static final int STATE_FOLLOW_WALL_TURN = 2;
  private static final int STATE_MAP_COMPLETE = 3;
  private static final int STATE_RALLY_MOVE = 4;
  private static final int STATE_RALLY_SPIN = 5;
  private static final int STATE_DONE = 6;

  //---VARIABLES---//
  private int state;
  private double moveStepDistance;
  private double turnStepAngle;
  private boolean pendingMove;
  private double lastMoveHeading;

  private double myX,myY;
  private double minX,maxX,minY,maxY;
  private double startX,startY,startHeading;
  private int cornerCount;
  private boolean loopComplete;
  private double turnReference;

  private double[][] rendezvousPoints;
  private int targetIndex;
  private double spinProgress;

  public BrainCanevas() { super(); }

  public void activate() {
    initKinematics();
    initOdometry();
    state = STATE_ALIGN_NORTH;
    sendLogMessage("Activation completed. Preparing wall-following routine.");
  }
  public void step() {
    updateOdometry();

    switch(state) {
      case STATE_ALIGN_NORTH:
        if (alignTo(startHeading)) {
          state = STATE_FOLLOW_WALL_MOVE;
        }
        return;
      case STATE_FOLLOW_WALL_MOVE:
        if (loopComplete) {
          state = STATE_MAP_COMPLETE;
          return;
        }
        if (detectFront().getObjectType()==IFrontSensorResult.Types.NOTHING) {
          scheduleMove();
        } else {
          state = STATE_FOLLOW_WALL_TURN;
          turnReference = getHeading();
          stepTurn(Parameters.Direction.RIGHT);
        }
        return;
      case STATE_FOLLOW_WALL_TURN:
        if (isHeading(turnReference + Parameters.RIGHTTURNFULLANGLE)) {
          cornerCount++;
          state = STATE_FOLLOW_WALL_MOVE;
        } else {
          stepTurn(Parameters.Direction.RIGHT);
        }
        return;
      case STATE_MAP_COMPLETE:
        prepareRendezvous();
        state = STATE_RALLY_MOVE;
        return;
      case STATE_RALLY_MOVE:
        if (rendezvousPoints == null || targetIndex >= rendezvousPoints.length) {
          state = STATE_DONE;
          return;
        }
        if (reachTarget(rendezvousPoints[targetIndex][0], rendezvousPoints[targetIndex][1])) {
          state = STATE_RALLY_SPIN;
          spinProgress = 0;
          sendLogMessage("Reached rendezvous point #"+(targetIndex+1)+", performing 360° scan.");
        }
        return;
      case STATE_RALLY_SPIN:
        spinProgress += Math.abs(turnStepAngle);
        stepTurn(Parameters.Direction.LEFT);
        if (spinProgress >= 2*Math.PI) {
          targetIndex++;
          if (targetIndex >= rendezvousPoints.length) {
            state = STATE_DONE;
            sendLogMessage("All rendezvous objectives completed.");
          } else {
            state = STATE_RALLY_MOVE;
          }
        }
        return;
      case STATE_DONE:
      default:
        // Nothing to do, keep monitoring sensors.
        return;
    }
  }

  //---PRIVATE-HELPERS---//
  private void initKinematics() {
    boolean isMainBot = getHealth() > 150.0; //main bots have 300 HP, secondary 100 HP
    moveStepDistance = isMainBot ? Parameters.teamAMainBotSpeed : Parameters.teamASecondaryBotSpeed;
    turnStepAngle = isMainBot ? Parameters.teamAMainBotStepTurnAngle : Parameters.teamASecondaryBotStepTurnAngle;
  }

  private void initOdometry() {
    myX = myY = 0.0;
    minX = maxX = myX;
    minY = maxY = myY;
    startX = myX;
    startY = myY;
    startHeading = Parameters.NORTH;
    pendingMove = false;
    lastMoveHeading = getHeading();
    cornerCount = 0;
    loopComplete = false;
    rendezvousPoints = null;
    targetIndex = 0;
    spinProgress = 0;
  }

  private void updateOdometry() {
    if (!pendingMove) return;
    pendingMove = false;
    myX += moveStepDistance * Math.cos(lastMoveHeading);
    myY += moveStepDistance * Math.sin(lastMoveHeading);
    minX = Math.min(minX,myX);
    maxX = Math.max(maxX,myX);
    minY = Math.min(minY,myY);
    maxY = Math.max(maxY,myY);

    if (cornerCount >= 4 && isNear(startX,startY,myX,myY)) {
      if (isHeading(startHeading)) {
        if (!loopComplete) {
          loopComplete = true;
          double width = maxX - minX;
          double height = maxY - minY;
          sendLogMessage("Mapping completed. Estimated arena size: "
            + (int)Math.round(width) + "mm x " + (int)Math.round(height) + "mm.");
        }
      }
    }
  }

  private void scheduleMove() {
    pendingMove = true;
    lastMoveHeading = getHeading();
    move();
  }

  private boolean alignTo(double targetHeading) {
    if (isHeading(targetHeading)) return true;
    double delta = normalizeAngle(targetHeading - getHeading());
    if (delta > 0) stepTurn(Parameters.Direction.LEFT);
    else stepTurn(Parameters.Direction.RIGHT);
    return false;
  }

  private boolean isHeading(double direction) {
    return Math.abs(normalizeAngle(direction - getHeading())) < HEADINGPRECISION;
  }

  private double normalizeAngle(double angle) {
    double res = angle;
    while (res <= -Math.PI) res += 2*Math.PI;
    while (res > Math.PI) res -= 2*Math.PI;
    return res;
  }

  private boolean isNear(double ax,double ay,double bx,double by) {
    return Math.hypot(ax-bx, ay-by) < POSITIONPRECISION;
  }

  private void prepareRendezvous() {
    double width = maxX - minX;
    double southY = minY;
    double westX = minX;
    if (width <= 0) width = moveStepDistance; //fallback if mapping failed

    rendezvousPoints = new double[][] {
      {westX + 0.25*width, southY},
      {westX + 0.50*width, southY},
      {westX + 0.75*width, southY}
    };
    targetIndex = 0;
    sendLogMessage("Rendezvous coordinates prepared along southern border.");
  }

  private boolean reachTarget(double targetX,double targetY) {
    double dx = targetX - myX;
    double dy = targetY - myY;
    double dist = Math.hypot(dx, dy);
    if (dist < POSITIONPRECISION) return true;

    double desiredHeading = Math.atan2(dy, dx);
    double delta = normalizeAngle(desiredHeading - getHeading());
    if (Math.abs(delta) > HEADINGPRECISION) {
      if (delta > 0) stepTurn(Parameters.Direction.LEFT);
      else stepTurn(Parameters.Direction.RIGHT);
    } else {
      if (detectFront().getObjectType()==IFrontSensorResult.Types.NOTHING) {
        scheduleMove();
      } else {
        stepTurn(Parameters.Direction.LEFT);
      }
    }
    return false;
  }
}
