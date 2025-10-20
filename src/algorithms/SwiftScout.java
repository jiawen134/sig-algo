/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * GPLv3+ — SwiftScout (Fast skirmisher: arc-strafe, flank, reactive dodge)
 * ******************************************************/
package algorithms;

import java.util.ArrayList;
import java.lang.reflect.Method;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IRadarResult;
import characteristics.IFrontSensorResult;

public class SwiftScout extends Brain {

  // ====== 状态机 ======
  private enum Mode { SWEEP, ENGAGE, EVADE, AVOID }
  private Mode mode = Mode.SWEEP;

  // ====== 常量 ======
  private static final double TWO_PI = Math.PI * 2;
  private static final boolean RADAR_BEARING_IS_ABSOLUTE = false; // 如为绝对角，改为 true

  private static final double STEP_TURN =
      Math.max(Parameters.teamASecondaryBotStepTurnAngle, Parameters.teamBSecondaryBotStepTurnAngle);

  private static final double AIM_TOL     = STEP_TURN * 1.1; // 更灵活的对准阈值
  private static final double FIRE_CONE   = STEP_TURN * 3.2;
  private static final double SAFE_CONE   = STEP_TURN * 2.0;
  private static final int    LOCAL_RELOAD= Math.max(8, Parameters.bulletFiringLatency - 4); // 骚扰更激进

  private static final int LOST_TIMEOUT   = 40; // 视野短，容忍度低一些
  private static final int ORBIT_PERIOD   = 20; // 绕圈/蛇形节奏更快
  private static final int EVADE_TICKS    = 16; // 被击中更快脱离
  private static final int AVOID_STICK    = 12;

  // ====== 运行时变量 ======
  private boolean orbitRight = true; // 绕敌方向
  private boolean preferRight = true; // 扫描/避障偏向
  private int tick = 0, reload = 0, lostTicks = LOST_TIMEOUT + 1;
  private int orbitPhase = 0, avoidTicks = 0, evadeTicks = 0;
  private boolean avoidRight = true;

  private double targetDir = Double.NaN, lastSeenDir = Double.NaN;
  private double lastEnemyDir = Double.NaN, enemyAngVel = 0.0;
  private double lastHealth = -1, threatDir = Double.NaN;

  @Override
  public void activate() {
    mode = Mode.SWEEP;
    tick = 0; reload = 0; orbitPhase = 0;
    avoidTicks = 0; evadeTicks = 0;
    preferRight = true; orbitRight = true; avoidRight = true;
    targetDir = Double.NaN; lastSeenDir = getHeading();
    lastEnemyDir = Double.NaN; enemyAngVel = 0.0;
    lastHealth = getHealth(); threatDir = Double.NaN;
    sendLogMessage("SwiftScout: activated.");
  }

  @Override
  public void step() {
    tick++;
    if (reload > 0) reload--;

    // 0) HP -> EVADE
    double hp = getHealth();
    if (lastHealth >= 0 && hp < lastHealth) {
      double hitDir = tryGetLastHitDirection();
      if (Double.isNaN(hitDir)) hitDir = !Double.isNaN(lastSeenDir) ? lastSeenDir : normalizeAngle(getHeading() + Math.PI);
      threatDir = hitDir;
      evadeTicks = EVADE_TICKS;
      mode = Mode.EVADE;
      // 遭受命中时反向绕圈，提高脱离概率
      orbitRight = !orbitRight;
    }
    lastHealth = hp;

    // 1) 避障优先
    if (frontBlocked()) {
      mode = Mode.AVOID;
      if (avoidTicks == 0) { avoidRight = preferRight; preferRight = !preferRight; }
      avoidTicks = AVOID_STICK;
    } else if (mode == Mode.AVOID) {
      if (avoidTicks > 0) avoidTicks--;
      if (avoidTicks == 0) mode = Mode.SWEEP;
    }

    // 2) 选敌（非 AVOID 时）
    if (mode != Mode.AVOID) {
      ArrayList<IRadarResult> radar = detectRadar();
      IRadarResult lock = pickTarget(radar); // 优先主机，再副机——但副机会用绕圈与机动取胜

      if (lock != null) {
        double raw = lock.getObjectDirection();
        double cur = RADAR_BEARING_IS_ABSOLUTE ? normalizeAngle(raw) : normalizeAngle(getHeading() + raw);

        if (!Double.isNaN(lastEnemyDir)) {
          double d = signedAngleDiff(lastEnemyDir, cur);
          enemyAngVel = 0.6 * enemyAngVel + 0.4 * d;
        }
        lastEnemyDir = cur;

        targetDir = cur; lastSeenDir = cur;
        lostTicks = 0;
        if (mode != Mode.EVADE) mode = Mode.ENGAGE;
      } else {
        lostTicks++;
        if (lostTicks > LOST_TIMEOUT && mode != Mode.EVADE) {
          targetDir = Double.NaN; lastEnemyDir = Double.NaN; enemyAngVel = 0.0;
          mode = Mode.SWEEP;
        } else if (mode != Mode.EVADE) {
          targetDir = lastSeenDir;
          mode = Mode.ENGAGE;
        }
      }
    }

    // 3) 行为
    switch (mode) {
      case AVOID:  avoidStep();  break;
      case EVADE:  evadeStep();  break;
      case ENGAGE: engageStep(); break;
      case SWEEP:
      default:     sweepStep();  break;
    }
  }

  // ====== 行为 ======
  private void avoidStep() {
    stepTurn(avoidRight ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
    moveBack();
  }

  private void evadeStep() {
    // 相对威胁 ±90° 更近一侧
    double right = normalizeAngle(threatDir + 0.5 * Math.PI);
    double left  = normalizeAngle(threatDir - 0.5 * Math.PI);
    double dR = Math.abs(signedAngleDiff(getHeading(), right));
    double dL = Math.abs(signedAngleDiff(getHeading(), left));
    double goal = (dR < dL) ? right : left;

    double diff = signedAngleDiff(getHeading(), goal);
    if (Math.abs(diff) > AIM_TOL) {
      stepTurn(diff > 0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
      return;
    }

    // 侧移 + 偶发压制射击
    if (reload == 0 && !friendlyFront()) {
      double jitter = (Math.random() - 0.5) * AIM_TOL * 1.4;
      fire(normalizeAngle(threatDir + jitter));
      reload = LOCAL_RELOAD;
    }
    move(); // 快速侧移
    if (--evadeTicks <= 0) mode = Double.isNaN(targetDir) ? Mode.SWEEP : Mode.ENGAGE;
  }

  private void engageStep() {
    // 以目标为中心做“角度环绕”——保持侧翼，不与重坦正面硬刚
    // 目标角速度简易前导
    double lead = targetDir + enemyAngVel * 6.0; // 快节奏，前导略小
    lead = normalizeAngle(lead);

    // 先将机头朝向“切线方向”：目标角 ± 90°
    double tangent = normalizeAngle(lead + (orbitRight ? +0.5 : -0.5) * Math.PI);
    double diff = signedAngleDiff(getHeading(), tangent);
    if (Math.abs(diff) > AIM_TOL) {
      stepTurn(diff > 0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
      return;
    }

    // 在切线运动的同时朝 lead 开火（保持侧向机动）
    if (!friendlyFront() && reload == 0 &&
        Math.abs(signedAngleDiff(getHeading(), lead)) < FIRE_CONE) {
      double jitter = (Math.random() - 0.5) * AIM_TOL * 1.0;
      fire(normalizeAngle(lead + jitter));
      reload = LOCAL_RELOAD;
    }

    // 绕圈/蛇形推进
    orbitPhase = (orbitPhase + 1) % ORBIT_PERIOD;
    move(); // 速度快，持续前进更利于牵制
    // 每一小段反向一个 tick，形成小蛇形
    if (orbitPhase == ORBIT_PERIOD / 2) {
      stepTurn(orbitRight ? Parameters.Direction.LEFT : Parameters.Direction.RIGHT);
    }
  }

  private void sweepStep() {
    // 弧线巡航 + 周期变向，快速刷到侧向目标
    if (tick % 25 == 0) preferRight = !preferRight;
    stepTurn(preferRight ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
    move();
  }

  // ====== 传感/工具 ======
  private boolean friendlyFront() {
    try {
      Object fr = detectFront(); if (fr == null) return false;
      IFrontSensorResult.Types t = (fr instanceof IFrontSensorResult)
          ? ((IFrontSensorResult) fr).getObjectType()
          : (IFrontSensorResult.Types) fr;
      if (t == null) return false;
      String n = t.name();
      if (n.equalsIgnoreCase("Nothing")) return false;
      return n.startsWith("Team") || n.equalsIgnoreCase("WALL");
    } catch (Throwable e) { return false; }
  }

  private boolean frontBlocked() {
    try {
      Object fr = detectFront(); if (fr == null) return false;
      IFrontSensorResult.Types t = (fr instanceof IFrontSensorResult)
          ? ((IFrontSensorResult) fr).getObjectType()
          : (IFrontSensorResult.Types) fr;
      if (t == null) return false;
      String n = t.name();
      if (n.equalsIgnoreCase("Nothing")) return false;
      return true; // 墙/队友/敌人都视为需要规避（副机更怕贴脸）
    } catch (Throwable e) { return false; }
  }

  private IRadarResult pickTarget(ArrayList<IRadarResult> list) {
    if (list == null || list.isEmpty()) return null;
    IRadarResult sec = null;
    for (IRadarResult r : list) {
      if (r.getObjectType() == IRadarResult.Types.OpponentMainBot) return r;
      if (r.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) sec = r;
    }
    return sec;
  }

  private double tryGetLastHitDirection() {
    try {
      Method m = robotsimulator.Brain.class.getMethod("getLastHitDirection");
      Object v = m.invoke(this);
      if (v instanceof Double) return normalizeAngle((Double) v);
    } catch (Throwable ignore) {}
    return Double.NaN;
  }

  private static double signedAngleDiff(double a, double b) {
    double d = normalizeAngle(b - a);
    if (d > Math.PI)  d -= TWO_PI;
    if (d <= -Math.PI) d += TWO_PI;
    return d;
  }

  private static double normalizeAngle(double x) {
    x %= TWO_PI; if (x < 0) x += TWO_PI; return x;
  }
}
