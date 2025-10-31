package algorithms;

import java.util.*;
import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

/**
 * MagicSecondary v2.0 - 学习Stage1Secondary的核心优势
 * 1. 精确的位置追踪（速度3，每次移动立即更新）
 * 2. 初始机动（先朝南/北移动到预定位置）
 * 3. 智能避障（根据位置和朝向判断）
 * 4. 逃跑机制（检测到近距离敌人立即后退）
 */
public class MagicSecondary extends Brain {
	
  // ===== 常量 =====
  private static final double TWO_PI = Math.PI * 2.0;
  private static final double PI = Math.PI;
  private static final double HALF_PI = Math.PI / 2.0;
  
  private static final double FIELD_WIDTH = 3000.0;
  private static final double FIELD_HEIGHT = 2000.0;
  private static final double SECONDARY_BOT_SPEED = Parameters.teamASecondaryBotSpeed;  // 3
  private static final double BULLET_RANGE = Parameters.bulletRange;  // 1000
  
  // 通信参数
  private static final int POS_PERIOD = 20;
  private static final int MSG_TTL = 80;
  
  // 边界边距
  private static final double BOUNDARY_MARGIN = 50.0;
  private static final double CORNER_MARGIN = 200.0;
  
  // 威胁距离
  private static final double MAIN_THREAT_DISTANCE = 400.0;
  private static final double SECONDARY_THREAT_DISTANCE = 350.0;
  private static final double CLOSE_THREAT_DISTANCE = 120.0;
  
  // 状态机
  private static final int STATE_INITIAL_TURN_NORTH = 1;
  private static final int STATE_INITIAL_TURN_SOUTH = 2;
  private static final int STATE_INITIAL_MOVE = 3;
  private static final int STATE_TURN_TO_EAST = 4;
  private static final int STATE_TURN_TO_WEST = 5;
  private static final int STATE_NORMAL_MOVE = 6;
  private static final int STATE_FLEE = 7;
  private static final int STATE_RETREAT = 8;
  private static final int STATE_TURN_LEFT = 9;
  private static final int STATE_TURN_RIGHT = 10;
  
  // ===== 状态 =====
  private int tick = 0;
  private int lastPosBroadcast = -9999;
  private int state = STATE_INITIAL_TURN_NORTH;
  private int retreatStartTick = 0;
  private double targetTurnDirection = 0.0;
  private final String myId = "SCOUT_" + Integer.toHexString((int)(Math.random()*0xFFFF));
  
  // 位置追踪
  private double posX;
  private double posY;
  private boolean movingForward = false;
  private boolean movingBackward = false;
  private boolean isNorthBot = true;  // true=顶部(ROCKY), false=底部(MARIO)
  private boolean isLeftTeam = true;
  
  // 检测到的敌人（用于逃跑判断）
  private final List<IRadarResult> detectedThreats = new ArrayList<>();
  
  // ===== 生命周期 =====
  @Override
  public void activate() {
    tick = 0;
    lastPosBroadcast = -9999;
    
    // 自定位
    identifySelf();
    determineTeamSide();
    setInitialPosition();
    
    sendLogMessage("MagicSecondary");
  }
  
  @Override
  public void step() {
    tick++;
    
    // ===== 第一步：立即更新位置 =====
    updatePosition();
    
    // ===== 第二步：周期广播位置 =====
    if (tick - lastPosBroadcast >= POS_PERIOD) {
      lastPosBroadcast = tick;
      broadcast(String.format(Locale.ROOT, "POS|%s|%d|%.1f|%.1f|%.6f", 
          myId, tick, posX, posY, getHeading()));
    }
    
    // ===== 第三步：雷达扫描 =====
    detectedThreats.clear();
    ArrayList<IRadarResult> radarResults = detectRadar();
    
    if (radarResults != null) {
      for (IRadarResult r : radarResults) {
        if (r.getObjectType() == IRadarResult.Types.OpponentMainBot ||
            r.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
          
          // 广播敌人位置
          double enemyX = posX + r.getObjectDistance() * Math.cos(r.getObjectDirection());
          double enemyY = posY + r.getObjectDistance() * Math.sin(r.getObjectDirection());
          broadcast(String.format(Locale.ROOT, "ENEMY|%s|%d|%.1f|%.1f|%.1f", 
              myId, tick, enemyX, enemyY, r.getObjectRadius()));
          
          // 检测威胁
          boolean isMainThreat = r.getObjectType() == IRadarResult.Types.OpponentMainBot &&
                                 r.getObjectDistance() <= MAIN_THREAT_DISTANCE;
          boolean isSecondaryThreat = r.getObjectType() == IRadarResult.Types.OpponentSecondaryBot &&
                                      r.getObjectDistance() <= SECONDARY_THREAT_DISTANCE;
          
          if (isMainThreat || isSecondaryThreat) {
            detectedThreats.add(r);
            if (state == STATE_NORMAL_MOVE) {
              state = STATE_FLEE;
            }
          }
          
          // 非常近的威胁 - 立即后退
          if (r.getObjectDistance() < CLOSE_THREAT_DISTANCE && 
              r.getObjectType() != IRadarResult.Types.BULLET &&
              state == STATE_NORMAL_MOVE) {
            state = STATE_RETREAT;
            retreatStartTick = tick;
          }
        }
      }
    }
    
    // ===== 第四步：边界处理 =====
    if (handleBoundaries()) {
      return;
    }
    
    // ===== 第五步：状态机 =====
    executeStateMachine();
  }
  
  // ===== 位置更新 =====
  private void updatePosition() {
    if (movingForward) {
      posX += SECONDARY_BOT_SPEED * Math.cos(getHeading());
      posY += SECONDARY_BOT_SPEED * Math.sin(getHeading());
      clampCoordinates();
      movingForward = false;
    }
    
    if (movingBackward) {
      posX -= SECONDARY_BOT_SPEED * Math.cos(getHeading());
      posY -= SECONDARY_BOT_SPEED * Math.sin(getHeading());
      clampCoordinates();
      movingBackward = false;
    }
  }
  
  private void clampCoordinates() {
    if (posX < 0.0) posX = 0.0;
    if (posX > FIELD_WIDTH) posX = FIELD_WIDTH;
    if (posY < 0.0) posY = 0.0;
    if (posY > FIELD_HEIGHT) posY = FIELD_HEIGHT;
  }
  
  // ===== 自定位 =====
  private void identifySelf() {
    isNorthBot = true;  // 默认是北侧机器人(ROCKY)
    
    ArrayList<IRadarResult> rs = detectRadar();
    if (rs != null) {
      for (IRadarResult r : rs) {
        if (areDirectionsEqual(r.getObjectDirection(), -HALF_PI)) {
          isNorthBot = false;  // 检测到南侧有队友，说明自己是南侧(MARIO)
          break;
        }
      }
    }
  }
  
  private void determineTeamSide() {
    isLeftTeam = true;  // 默认是左侧队伍
    
    ArrayList<IRadarResult> rs = detectRadar();
    if (rs != null) {
      for (IRadarResult r : rs) {
        if (areDirectionsEqual(r.getObjectDirection(), 0.0)) {
          isLeftTeam = false;  // 检测到东侧有队友，说明自己在右侧
          break;
        }
      }
    }
  }
  
  private void setInitialPosition() {
    if (isLeftTeam) {
      if (isNorthBot) {
        posX = 500.0;
        posY = 800.0;
        state = STATE_INITIAL_TURN_NORTH;
      } else {
        posX = 500.0;
        posY = 1200.0;
        state = STATE_INITIAL_TURN_SOUTH;
      }
    } else {
      if (isNorthBot) {
        posX = 2500.0;
        posY = 800.0;
        state = STATE_INITIAL_TURN_NORTH;
      } else {
        posX = 2500.0;
        posY = 1200.0;
        state = STATE_INITIAL_TURN_SOUTH;
      }
    }
  }
  
  // ===== 边界处理 =====
  private boolean handleBoundaries() {
    if (posX <= BOUNDARY_MARGIN) {
      if (!isFacingDirection(0.0)) {
        turnToDirection(0.0);
        return true;
      }
      state = STATE_NORMAL_MOVE;
    }
    
    if (posX >= FIELD_WIDTH - BOUNDARY_MARGIN) {
      if (!isFacingDirection(PI)) {
        turnToDirection(PI);
        return true;
      }
      state = STATE_NORMAL_MOVE;
    }
    
    if (posY <= BOUNDARY_MARGIN) {
      if (!isFacingDirection(HALF_PI)) {
        turnToDirection(HALF_PI);
        return true;
      }
      state = STATE_NORMAL_MOVE;
    }
    
    if (posY >= FIELD_HEIGHT - BOUNDARY_MARGIN) {
      if (!isFacingDirection(-HALF_PI)) {
        turnToDirection(-HALF_PI);
        return true;
      }
      state = STATE_NORMAL_MOVE;
    }
    
    return false;
  }
  
  // ===== 状态机 =====
  private void executeStateMachine() {
    switch (state) {
      case STATE_INITIAL_TURN_NORTH:
        handleInitialTurnNorth();
        break;
      case STATE_INITIAL_TURN_SOUTH:
        handleInitialTurnSouth();
        break;
      case STATE_INITIAL_MOVE:
        handleInitialMove();
        break;
      case STATE_TURN_TO_EAST:
      case STATE_TURN_TO_WEST:
        handleTurnToDirection();
        break;
      case STATE_NORMAL_MOVE:
        handleNormalMove();
        break;
      case STATE_TURN_LEFT:
      case STATE_TURN_RIGHT:
        handleTurnState();
        break;
      case STATE_RETREAT:
        handleRetreat();
        break;
      case STATE_FLEE:
        handleFlee();
        break;
      default:
        performMove();
    }
  }
  
  private void handleInitialTurnNorth() {
    if (!isFacingDirection(-HALF_PI)) {
      turnToDirection(-HALF_PI);
    } else {
      state = STATE_INITIAL_MOVE;
      performMove();
    }
  }
  
  private void handleInitialTurnSouth() {
    if (!isFacingDirection(HALF_PI)) {
      turnToDirection(HALF_PI);
    } else {
      state = STATE_INITIAL_MOVE;
      performMove();
    }
  }
  
  private void handleInitialMove() {
    performMove();
    
    if (isNorthBot) {
      if (posY < 500.0) {
        state = isLeftTeam ? STATE_TURN_TO_EAST : STATE_TURN_TO_WEST;
      }
    } else {
      if (posY > 1800.0) {
        state = isLeftTeam ? STATE_TURN_TO_EAST : STATE_TURN_TO_WEST;
      }
    }
  }
  
  private void handleTurnToDirection() {
    double targetDir = (state == STATE_TURN_TO_EAST) ? 0.0 : PI;
    if (!isFacingDirection(targetDir)) {
      turnToDirection(targetDir);
    } else {
      state = STATE_NORMAL_MOVE;
      performMove();
    }
  }
  
  private void handleNormalMove() {
    IFrontSensorResult.Types frontObj = frontType();
    
    if (frontObj == IFrontSensorResult.Types.WALL) {
      handleWallCollision();
      return;
    }
    
    if (frontObj == IFrontSensorResult.Types.Wreck ||
        frontObj == IFrontSensorResult.Types.TeamMainBot ||
        frontObj == IFrontSensorResult.Types.TeamSecondaryBot) {
      state = STATE_TURN_LEFT;
      targetTurnDirection = getHeading() - HALF_PI;
      stepTurn(Parameters.Direction.LEFT);
      return;
    }
    
    performMove();
  }
  
  private void handleWallCollision() {
    // 检测角落
    boolean atCorner = (posX > FIELD_WIDTH - CORNER_MARGIN && posY > FIELD_HEIGHT - CORNER_MARGIN) ||
                       (posX > FIELD_WIDTH - CORNER_MARGIN && posY < CORNER_MARGIN) ||
                       (posX < CORNER_MARGIN && posY < CORNER_MARGIN) ||
                       (posX < CORNER_MARGIN && posY > FIELD_HEIGHT - CORNER_MARGIN);
    
    if (atCorner) {
      state = STATE_TURN_LEFT;
      targetTurnDirection = getHeading() - HALF_PI;
      stepTurn(Parameters.Direction.LEFT);
      return;
    }
    
    // 中间区域检测
    boolean midHorizontal = !(posX > FIELD_WIDTH - CORNER_MARGIN) && !(posX < CORNER_MARGIN);
    boolean midVertical = !(posY > FIELD_HEIGHT - CORNER_MARGIN) && !(posY < CORNER_MARGIN);
    
    if (midHorizontal && midVertical) {
      performMove();
      return;
    }
    
    if (midHorizontal) {
      if (!isFacingDirection(-HALF_PI) && !isFacingDirection(HALF_PI)) {
        performMove();
        return;
      }
    }
    
    if (!isFacingDirection(0.0) && !isFacingDirection(PI)) {
      performMove();
      return;
    }
    
    state = STATE_TURN_LEFT;
    targetTurnDirection = getHeading() - HALF_PI;
    stepTurn(Parameters.Direction.LEFT);
  }
  
  private void handleTurnState() {
    if (isFacingDirection(targetTurnDirection)) {
      state = STATE_NORMAL_MOVE;
      performMove();
    } else {
      Parameters.Direction dir = (state == STATE_TURN_LEFT) ? 
          Parameters.Direction.LEFT : Parameters.Direction.RIGHT;
      stepTurn(dir);
    }
  }
  
  private void handleRetreat() {
    if (tick < retreatStartTick + 25) {
      performMoveBack();
    } else {
      // 后退完成，随机转向
      if (Math.random() < 0.5) {
        state = STATE_TURN_LEFT;
        targetTurnDirection = getHeading() - HALF_PI;
        stepTurn(Parameters.Direction.LEFT);
      } else {
        state = STATE_TURN_RIGHT;
        targetTurnDirection = getHeading() + HALF_PI;
        stepTurn(Parameters.Direction.RIGHT);
      }
    }
  }
  
  private void handleFlee() {
    // 检测是否在角落附近
    boolean nearCorner = (posX > FIELD_WIDTH - 100.0 || posX < 100.0) &&
                         (posY > FIELD_HEIGHT - 100.0 || posY < 100.0);
    
    if (nearCorner) {
      state = STATE_TURN_RIGHT;
      targetTurnDirection = getHeading() + HALF_PI;
      stepTurn(Parameters.Direction.RIGHT);
      return;
    }
    
    // 中间区域检测
    boolean midHorizontal = !(posX > FIELD_WIDTH - 100.0) && !(posX < 100.0);
    boolean midVertical = !(posY > FIELD_HEIGHT - 100.0) && !(posY < 100.0);
    
    if (midHorizontal && midVertical) {
      performFleeMove();
      return;
    }
    
    if (midHorizontal) {
      if (!isFacingDirection(-HALF_PI) && !isFacingDirection(HALF_PI)) {
        performFleeMove();
        return;
      }
    }
    
    if (!isFacingDirection(0.0) && !isFacingDirection(PI)) {
      performFleeMove();
      return;
    }
    
    state = STATE_TURN_RIGHT;
    targetTurnDirection = getHeading() + HALF_PI;
    stepTurn(Parameters.Direction.RIGHT);
  }
  
  private void performFleeMove() {
    moveBack();
    posX -= SECONDARY_BOT_SPEED * Math.cos(getHeading());
    posY -= SECONDARY_BOT_SPEED * Math.sin(getHeading());
    clampCoordinates();
    
    if (detectedThreats.isEmpty()) {
      state = STATE_NORMAL_MOVE;
    }
  }
  
  // ===== 移动操作 =====
  private void performMove() {
    movingForward = true;
    move();
  }
  
  private void performMoveBack() {
    movingBackward = true;
    moveBack();
  }
  
  // ===== 工具函数 =====
  private IFrontSensorResult.Types frontType() {
    try {
      IFrontSensorResult r = detectFront();
      return (r == null) ? null : r.getObjectType();
    } catch (Throwable t) {
      return null;
    }
  }
  
  private void turnToDirection(double targetDir) {
    double heading = normalizeAngle(getHeading());
    
    if (targetDir == 0.0 || targetDir == PI) {
      if (heading < PI && heading > 0.0) {
        stepTurn(targetDir == 0.0 ? Parameters.Direction.LEFT : Parameters.Direction.RIGHT);
      } else {
        stepTurn(targetDir == 0.0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
      }
    } else {
      boolean inLowerHalf = !(heading < HALF_PI) && !(heading > 3 * HALF_PI);
      if (targetDir == -HALF_PI) {
        stepTurn(inLowerHalf ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
      } else {
        stepTurn(inLowerHalf ? Parameters.Direction.LEFT : Parameters.Direction.RIGHT);
      }
    }
  }
  
  private boolean isFacingDirection(double dir) {
    return Math.abs(Math.sin(getHeading() - dir)) < 0.01;
  }
  
  private boolean areDirectionsEqual(double dir1, double dir2) {
    return Math.abs(dir1 - dir2) < 0.001;
  }
  
  private double normalizeAngle(double angle) {
    while (angle < 0.0) angle += TWO_PI;
    while (angle >= TWO_PI) angle -= TWO_PI;
    return angle;
  }
}
