/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * GPLv3+ — AegisMain (Heavy anchor: stable aim, suppressive fire, reactive evasion)
 * ******************************************************/
package algorithms;

import java.util.ArrayList;
import java.lang.reflect.Method;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IRadarResult;
import characteristics.IFrontSensorResult;

public class AegisMain extends Brain {

  // ====== 状态机 ======
  private enum Mode { PATROL, ENGAGE, EVADE, AVOID }
  private Mode mode = Mode.PATROL;

  // ====== 常量（算法内，不改全局参数） ======
  private static final double TWO_PI = Math.PI * 2;
  private static final boolean RADAR_BEARING_IS_ABSOLUTE = false; // 如雷达给绝对角，改为 true

  private static final double STEP_TURN =
      Math.max(Parameters.teamAMainBotStepTurnAngle, Parameters.teamBMainBotStepTurnAngle);

  private static final double AIM_TOL     = STEP_TURN * 1.2; // 对准阈值
  private static final double FIRE_CONE   = STEP_TURN * 3.8; // 开火宽容角
  private static final double SAFE_CONE   = STEP_TURN * 2.2; // 友军正前方抑制射击
  private static final int    LOCAL_RELOAD= Parameters.bulletFiringLatency;

  private static final int LOST_TIMEOUT   = 60; // 主机视野长，可更耐心追向
  private static final int KITE_PERIOD    = 28; // 慢一些的风筝节奏
  private static final int EVADE_TICKS    = 22; // 被击中后侧向规避时间
  private static final int AVOID_STICK    = 16; // 避障粘性

  // ====== 运行时变量 ======
  private boolean preferRight = true;
  private int tick = 0, reload = 0, lostTicks = LOST_TIMEOUT + 1;
  private int kitePhase = 0, avoidTicks = 0, evadeTicks = 0;
  private boolean avoidRight = true;

  private double targetDir = Double.NaN;   // 锁定目标（绝对角）
  private double lastSeenDir = Double.NaN; // 最近一次可见方向
  private double lastEnemyDir = Double.NaN, enemyAngVel = 0.0;

  private double lastHealth = -1;          // HP 追踪
  private double threatDir = Double.NaN;   // 最近一次被击中推断的威胁方向

  @Override
  public void activate() {
    mode = Mode.PATROL;
    tick = 0; reload = 0; kitePhase = 0;
    avoidTicks = 0; evadeTicks = 0;
    preferRight = true; avoidRight = true;
    targetDir = Double.NaN; lastSeenDir = getHeading();
    lastEnemyDir = Double.NaN; enemyAngVel = 0.0;
    lastHealth = getHealth();
    threatDir = Double.NaN;
    sendLogMessage("AegisMain: activated.");
  }

  @Override
  public void step() {
    tick++;
    if (reload > 0) reload--;

    // 0) HP 检查：被击中则进入 EVADE，并记录威胁方向
    double hp = getHealth();
    if (lastHealth >= 0 && hp < lastHealth) {
      double hitDir = tryGetLastHitDirection(); // 优先真实命中方向
      if (Double.isNaN(hitDir)) hitDir = !Double.isNaN(lastSeenDir) ? lastSeenDir : normalizeAngle(getHeading() + Math.PI);
      threatDir = hitDir;
      evadeTicks = EVADE_TICKS;
      mode = Mode.EVADE;
    }
    lastHealth = hp;

    // 1) 避障优先
    if (frontBlocked()) {
      mode = Mode.AVOID;
      if (avoidTicks == 0) { avoidRight = preferRight; preferRight = !preferRight; }
      avoidTicks = AVOID_STICK;
    } else if (mode == Mode.AVOID) {
      if (avoidTicks > 0) avoidTicks--;
      if (avoidTicks == 0) mode = Mode.PATROL;
    }

    // 2) 侦察/选敌（非 AVOID 时进行）
    if (mode != Mode.AVOID) {
      ArrayList<IRadarResult> radar = detectRadar();
      IRadarResult lock = pickTarget(radar); // 优先主机，再副机

      if (lock != null) {
        double raw = lock.getObjectDirection();
        double cur = RADAR_BEARING_IS_ABSOLUTE ? normalizeAngle(raw) : normalizeAngle(getHeading() + raw);

        if (!Double.isNaN(lastEnemyDir)) {
          double d = signedAngleDiff(lastEnemyDir, cur); // 每 tick 的角速率
          // 指数平滑一点
          enemyAngVel = 0.7 * enemyAngVel + 0.3 * d;
        }
        lastEnemyDir = cur;

        targetDir = cur; lastSeenDir = cur;
        lostTicks = 0;
        // 若正在规避，保持 EVADE；否则进入 ENGAGE
        if (mode != Mode.EVADE) mode = Mode.ENGAGE;
      } else {
        lostTicks++;
        if (lostTicks > LOST_TIMEOUT && mode != Mode.EVADE) {
          targetDir = Double.NaN; lastEnemyDir = Double.NaN; enemyAngVel = 0.0;
          mode = Mode.PATROL;
        } else if (mode != Mode.EVADE) {
          // 短暂丢失：沿 lastSeen 方向保持朝向
          targetDir = lastSeenDir;
          mode = Mode.ENGAGE;
        }
      }
    }

    // 3) 执行行为
    switch (mode) {
      case AVOID:   avoidStep();   break;
      case EVADE:   evadeStep();   break;
      case ENGAGE:  engageStep();  break;
      case PATROL:
      default:      patrolStep();  break;
    }
  }

  // ====== 行为实现 ======
  private void avoidStep() {
    stepTurn(avoidRight ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
    moveBack();
  }

  private void evadeStep() {
    // 侧向风筝：相对威胁方向 ±90° 中更近的一侧
    double right = normalizeAngle(threatDir + 0.5 * Math.PI);
    double left  = normalizeAngle(threatDir - 0.5 * Math.PI);
    double dR = Math.abs(signedAngleDiff(getHeading(), right));
    double dL = Math.abs(signedAngleDiff(getHeading(), left));
    double goal = (dR < dL) ? right : left;

    double diff = signedAngleDiff(getHeading(), goal);
    if (Math.abs(diff) > AIM_TOL) {
      stepTurn(diff > 0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
      return; // 本 tick 只转不走
    }

    // 侧移并进行压制射击（若前方无友军/障碍）
    maybeSuppressFire(threatDir);

    move(); // 侧移出火线
    if (--evadeTicks <= 0) {
      // 若仍无目标，则回 PATROL；否则回 ENGAGE
      mode = Double.isNaN(targetDir) ? Mode.PATROL : Mode.ENGAGE;
    }
  }

  private void engageStep() {
    // 基于角速度做一点“角度前导”
    double lead = targetDir + enemyAngVel * 8.0; // 8 tick 经验前导（无距离信息下的经验值）
    lead = normalizeAngle(lead);

    double diff = signedAngleDiff(getHeading(), lead);
    if (Math.abs(diff) > AIM_TOL) {
      stepTurn(diff > 0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
      return; // 先把枪线对稳
    }

    // 友军/障碍在正前方 → 抑制射击
    if (!friendlyFront()) {
      if (reload == 0 && Math.abs(signedAngleDiff(getHeading(), lead)) < FIRE_CONE) {
        double jitter = (Math.random() - 0.5) * AIM_TOL * 0.9;
        fire(normalizeAngle(lead + jitter));
        reload = LOCAL_RELOAD;
      }
    }

    // 重坦式风筝：前后交替，小步进逼或拉开
    kitePhase = (kitePhase + 1) % KITE_PERIOD;
    if (kitePhase < KITE_PERIOD / 2) move(); else moveBack();
  }

  private void patrolStep() {
    // 弧线巡航 + 周期换边
    if (tick % 40 == 0) preferRight = !preferRight;
    stepTurn(preferRight ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
    move();
  }

  // 压制射击（朝威胁方向小抖动点射）
  private void maybeSuppressFire(double towardDir) {
    if (reload == 0 && !friendlyFront()) {
      double jitter = (Math.random() - 0.5) * AIM_TOL * 1.2;
      fire(normalizeAngle(towardDir + jitter));
      reload = LOCAL_RELOAD;
    }
  }

  // ====== 传感/工具 ======

  /** 友军/墙在正前方？若是则抑制射击 */
  private boolean friendlyFront() {
    try {
      Object fr = detectFront();
      if (fr == null) return false;
      IFrontSensorResult.Types t = (fr instanceof IFrontSensorResult)
          ? ((IFrontSensorResult) fr).getObjectType()
          : (IFrontSensorResult.Types) fr;
      if (t == null) return false;
      String n = t.name();
      if (n.equalsIgnoreCase("Nothing")) return false;
      // Team* 或 WALL 认为前方不宜开火
      return n.startsWith("Team") || n.equalsIgnoreCase("WALL");
    } catch (Throwable e) { return false; }
  }

  /** 前方是否被墙/队友/敌人占据（用于机动避障） */
  private boolean frontBlocked() {
    try {
      Object fr = detectFront();
      if (fr == null) return false;
      IFrontSensorResult.Types t = (fr instanceof IFrontSensorResult)
          ? ((IFrontSensorResult) fr).getObjectType()
          : (IFrontSensorResult.Types) fr;
      if (t == null) return false;
      String n = t.name();
      if (n.equalsIgnoreCase("Nothing")) return false;
      // 任何命中（墙/队友/敌人）都视为需要规避
      return true;
    } catch (Throwable e) { return false; }
  }

  /** 选敌：优先主机，其次副机 */
  private IRadarResult pickTarget(ArrayList<IRadarResult> list) {
    if (list == null || list.isEmpty()) return null;
    IRadarResult sec = null;
    for (IRadarResult r : list) {
      if (r.getObjectType() == IRadarResult.Types.OpponentMainBot) return r;
      if (r.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) sec = r;
    }
    return sec;
  }

  /** 反射获取“被击中方向”（若引擎提供）；失败则 NaN */
  private double tryGetLastHitDirection() {
    try {
      Method m = robotsimulator.Brain.class.getMethod("getLastHitDirection");
      Object v = m.invoke(this);
      if (v instanceof Double) return normalizeAngle((Double) v);
    } catch (Throwable ignore) {}
    return Double.NaN;
  }

  /** (b - a) 的最小有符号角度差，(-PI, PI] */
  private static double signedAngleDiff(double a, double b) {
    double d = normalizeAngle(b - a);
    if (d > Math.PI)  d -= TWO_PI;
    if (d <= -Math.PI) d += TWO_PI;
    return d;
  }

  /** 归一化到 [0, 2π) */
  private static double normalizeAngle(double x) {
    x %= TWO_PI; if (x < 0) x += TWO_PI; return x;
  }
}
