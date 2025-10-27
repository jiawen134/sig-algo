/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * $Id: algorithms/Stage1.java 2014-10-18 buixuan.
 * ******************************************************/
package algorithms;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumSet;
import java.util.List;
import java.util.Random;

public class Stage1SecondaryA extends Brain {
  //---PARAMETERS---//
  private static final double ANGLEPRECISION = 0.1;
  private static final double TARGET_ALIGNMENT_THRESHOLD = Math.toRadians(10);
  private static final double OBSTACLE_ALERT_DISTANCE = Parameters.teamASecondaryBotSpeed * 2.5;
  private static final double CLOSE_PROXIMITY_DISTANCE = Parameters.teamASecondaryBotSpeed * 1.2;
  private static final int STATE_TIMEOUT_STEPS = 40;

  private static final int ROCKY = 0x1EADDA;
  private static final int MARIO = 0x5EC0;

  //---VARIABLES---//
  private enum State {
    EXPLORING,
    APPROACHING_TARGET,
    EVADE
  }

  private enum Action {
    MOVE_FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    MOVE_BACKWARD,
    WAIT
  }

  private static class LocalObject {
    final IRadarResult.Types type;
    final double direction;
    final double distance;

    LocalObject(IRadarResult.Types type, double direction, double distance) {
      this.type = type;
      this.direction = direction;
      this.distance = distance;
    }
  }

  private State currentState;
  private int stateStepCounter;
  private double myX,myY;
  private boolean isMoving;
  private boolean lastMoveBackward;
  private int whoAmI;
  private final List<LocalObject> localObstacles = new ArrayList<LocalObject>();
  private final List<LocalObject> localTargets = new ArrayList<LocalObject>();
  private final Random random = new Random();
  private Action queuedAction = null;

  //---CONSTRUCTORS---//
  public Stage1SecondaryA() { super(); }

  //---ABSTRACT-METHODS-IMPLEMENTATION---//
  public void activate() {
    //ODOMETRY CODE
    whoAmI = ROCKY;
    for (IRadarResult o: detectRadar())
      if (isSameDirection(o.getObjectDirection(),Parameters.NORTH)) whoAmI=MARIO;
    if (whoAmI == ROCKY){
      myX=Parameters.teamASecondaryBot1InitX;
      myY=Parameters.teamASecondaryBot1InitY;
    } else {
      myX=0;
      myY=0;
    }

    //INIT
    currentState = State.EXPLORING;
    stateStepCounter = 0;
    isMoving=false;
    lastMoveBackward=false;
    localObstacles.clear();
    localTargets.clear();
    sendLogMessage("#" + getIdentityLabel() + " activating with state=" + currentState);
  }
  public void step() {
    updatePerception();
    updateStateMachine();

    if (queuedAction != null) {
      Action action = queuedAction;
      queuedAction = null;
      executeAction(action);
      return;
    }

    Action action = selectAction();
    executeAction(action);
  }
  private void myMove(){
    isMoving=true;
    lastMoveBackward=false;
    move();
  }
  private void myMoveBack(){
    isMoving=true;
    lastMoveBackward=true;
    moveBack();
  }
  private boolean isSameDirection(double dir1, double dir2){
    return Math.abs(normalize(dir1)-normalize(dir2))<ANGLEPRECISION;
  }
  private double normalize(double dir){
    double res=dir;
    while (res<0) res+=2*Math.PI;
    while (res>=2*Math.PI) res-=2*Math.PI;
    return res;
  }

  private void updatePerception() {
    if (isMoving && whoAmI == ROCKY){
      double direction = getHeading();
      double distance = Parameters.teamASecondaryBotSpeed * (lastMoveBackward ? -1 : 1);
      myX+=distance*Math.cos(direction);
      myY+=distance*Math.sin(direction);
      isMoving=false;
    }

    localObstacles.clear();
    localTargets.clear();

    for (IRadarResult radarResult : detectRadar()) {
      LocalObject obj = new LocalObject(
          radarResult.getObjectType(),
          normalize(radarResult.getObjectDirection()),
          radarResult.getObjectDistance());
      if (isTargetType(obj.type)) {
        localTargets.add(obj);
      }
      if (isObstacleType(obj.type)) {
        localObstacles.add(obj);
      }
    }

    if (whoAmI == ROCKY) {
      sendLogMessage(
          "#ROCKY pos_estimate (" + (int)myX + ", " + (int)myY + ") targets=" + localTargets.size() +
          " obstacles=" + localObstacles.size());
    }
  }

  private void updateStateMachine() {
    State previousState = currentState;
    boolean immediateThreat = hasImmediateThreat();
    boolean hasTargets = !localTargets.isEmpty();

    switch (currentState) {
      case EXPLORING:
        if (immediateThreat) {
          currentState = State.EVADE;
        } else if (hasTargets) {
          currentState = State.APPROACHING_TARGET;
        }
        break;
      case APPROACHING_TARGET:
        if (immediateThreat) {
          currentState = State.EVADE;
        } else if (!hasTargets) {
          currentState = State.EXPLORING;
        }
        break;
      case EVADE:
        if (!immediateThreat) {
          currentState = hasTargets ? State.APPROACHING_TARGET : State.EXPLORING;
        }
        break;
      default:
        currentState = State.EXPLORING;
    }

    if (previousState != currentState) {
      stateStepCounter = 0;
      sendLogMessage("#" + getIdentityLabel() + " state=" + previousState + " -> " + currentState);
    } else {
      stateStepCounter++;
      if (stateStepCounter > STATE_TIMEOUT_STEPS) {
        sendLogMessage("#" + getIdentityLabel() + " timeout in state=" + currentState + ", re-evaluating direction");
        introduceRandomPerturbation();
        stateStepCounter = 0;
      }
    }
  }

  private Action selectAction() {
    LocalObject nearestTarget = getNearest(localTargets);
    LocalObject nearestObstacle = getNearest(localObstacles);
    double heading = normalize(getHeading());

    switch (currentState) {
      case APPROACHING_TARGET:
        if (nearestTarget != null) {
          double targetDiff = angularDifference(heading, nearestTarget.direction);
          if (Math.abs(targetDiff) < TARGET_ALIGNMENT_THRESHOLD && !isFrontBlocked()) {
            return Action.MOVE_FORWARD;
          }
          return (targetDiff > 0) ? Action.TURN_LEFT : Action.TURN_RIGHT;
        }
        return Action.MOVE_FORWARD;
      case EVADE:
        if (isFrontBlocked() || (nearestObstacle != null && nearestObstacle.distance < CLOSE_PROXIMITY_DISTANCE)) {
          // Determine if turning left or right offers more clearance
          return chooseSaferTurn();
        }
        return Action.MOVE_BACKWARD;
      case EXPLORING:
      default:
        if (!isFrontBlocked()) {
          return Action.MOVE_FORWARD;
        }
        return chooseSaferTurn();
    }
  }

  private void executeAction(Action action) {
    switch (action) {
      case MOVE_FORWARD:
        sendLogMessage("#" + getIdentityLabel() + " action=MOVE_FORWARD state=" + currentState);
        myMove();
        break;
      case MOVE_BACKWARD:
        sendLogMessage("#" + getIdentityLabel() + " action=MOVE_BACKWARD state=" + currentState);
        myMoveBack();
        break;
      case TURN_LEFT:
        sendLogMessage("#" + getIdentityLabel() + " action=TURN_LEFT state=" + currentState);
        stepTurn(Parameters.Direction.LEFT);
        break;
      case TURN_RIGHT:
        sendLogMessage("#" + getIdentityLabel() + " action=TURN_RIGHT state=" + currentState);
        stepTurn(Parameters.Direction.RIGHT);
        break;
      case WAIT:
      default:
        sendLogMessage("#" + getIdentityLabel() + " action=WAIT state=" + currentState);
        break;
    }
  }

  private boolean isTargetType(IRadarResult.Types type) {
    return EnumSet.of(IRadarResult.Types.OpponentMainBot, IRadarResult.Types.OpponentSecondaryBot).contains(type);
  }

  private boolean isObstacleType(IRadarResult.Types type) {
    return type == IRadarResult.Types.Wreck
        || type == IRadarResult.Types.TeamMainBot
        || type == IRadarResult.Types.TeamSecondaryBot
        || type == IRadarResult.Types.BULLET
        || type == IRadarResult.Types.OpponentMainBot
        || type == IRadarResult.Types.OpponentSecondaryBot;
  }

  private LocalObject getNearest(List<LocalObject> objects) {
    if (objects.isEmpty()) {
      return null;
    }
    return Collections.min(objects, new Comparator<LocalObject>() {
      public int compare(LocalObject a, LocalObject b) {
        return Double.compare(a.distance, b.distance);
      }
    });
  }

  private boolean isFrontBlocked() {
    IFrontSensorResult front = detectFront();
    if (front.getObjectType() != IFrontSensorResult.Types.NOTHING) {
      return true;
    }
    for (LocalObject obstacle : localObstacles) {
      double diff = Math.abs(angularDifference(getHeading(), obstacle.direction));
      if (diff < TARGET_ALIGNMENT_THRESHOLD && obstacle.distance < OBSTACLE_ALERT_DISTANCE) {
        return true;
      }
    }
    return false;
  }

  private boolean hasImmediateThreat() {
    if (isFrontBlocked()) {
      return true;
    }
    for (LocalObject obstacle : localObstacles) {
      if (obstacle.distance < CLOSE_PROXIMITY_DISTANCE) {
        return true;
      }
    }
    return false;
  }

  private Action chooseSaferTurn() {
    double leftScore = 0;
    double rightScore = 0;
    for (LocalObject obstacle : localObstacles) {
      double diff = angularDifference(getHeading(), obstacle.direction);
      double weight = (OBSTACLE_ALERT_DISTANCE - obstacle.distance);
      if (weight < 0) {
        weight = 0;
      }
      if (diff > 0) {
        leftScore += weight;
      } else {
        rightScore += weight;
      }
    }
    if (Math.abs(leftScore - rightScore) < 0.01) {
      return random.nextBoolean() ? Action.TURN_LEFT : Action.TURN_RIGHT;
    }
    return leftScore < rightScore ? Action.TURN_LEFT : Action.TURN_RIGHT;
  }

  private double angularDifference(double from, double to) {
    double diff = normalize(to) - normalize(from);
    while (diff > Math.PI) diff -= 2 * Math.PI;
    while (diff < -Math.PI) diff += 2 * Math.PI;
    return diff;
  }

  private void introduceRandomPerturbation() {
    queuedAction = random.nextBoolean()
        ? (random.nextBoolean() ? Action.TURN_LEFT : Action.TURN_RIGHT)
        : Action.WAIT;
  }

  private String getIdentityLabel() {
    return (whoAmI == ROCKY) ? "ROCKY" : (whoAmI == MARIO ? "MARIO" : "UNKNOWN");
  }
}
