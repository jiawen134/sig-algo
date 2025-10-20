/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * GPLv3+ — Refactored BB2 (FSM + Stable aiming + Obstacle avoidance)
 * ******************************************************/
package algorithms;

import java.util.ArrayList;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IRadarResult;
import characteristics.IFrontSensorResult;

public class BB2 extends Brain {

  // ================== 状态机 ==================
  private enum Mode { SEARCH, ENGAGE, AVOID }
  private Mode mode = Mode.SEARCH;

  // ================== 算法内部“常量” ==================
  private static final double TWO_PI = Math.PI * 2;

  // 取两队定义里较大的单步转角，确保所有机体上都稳定
  private static final double STEP_TURN =
      Math.max(Parameters.teamAMainBotStepTurnAngle, Parameters.teamBMainBotStepTurnAngle);

  // “已对准”阈值与宽容开火角
  private static final double AIM_TOL   = STEP_TURN * 1.2;
  private static final double FIRE_CONE = STEP_TURN * 3.5;

  // 目标丢失 tick 数：超过则回 SEARCH
  private static final int LOST_TIMEOUT = 50;

  // 轻量风筝（前/后交替）周期
  private static final int KITE_PERIOD = 24;

  // 本地射击节流（与底层冷却叠加）
  private static final int LOCAL_RELOAD = Parameters.bulletFiringLatency;

  // 避障持续 tick（粘性，避免“顶墙抖动”）
  private static final int AVOID_STICK  = 16;

  // 雷达角度是否为绝对角；若你的实现返回绝对角，改为 true
  private static final boolean RADAR_BEARING_IS_ABSOLUTE = false;

  // ================== 运行时变量 ==================
  private boolean preferRight = true;     // 搜索/避障时的初始偏向
  private int tick = 0;
  private int reload = 0;
  private int lostTicks = LOST_TIMEOUT + 1;
  private int kitePhase = 0;

  private int avoidTicks = 0;             // 避障剩余计数
  private boolean avoidRight = true;      // 本次避障向右/左转

  private double targetDir = Double.NaN;  // 当前锁定目标绝对方向
  private double lastSeenDir = 0;         // 最近一次可见时的绝对方向

  // ================== 生命周期 ==================
  @Override
  public void activate() {
    mode = Mode.SEARCH;
    tick = 0;
    reload = 0;
    lostTicks = LOST_TIMEOUT + 1;
    kitePhase = 0;
    avoidTicks = 0;
    preferRight = true;
    targetDir = Double.NaN;
    lastSeenDir = getHeading();
    sendLogMessage("BB2 v3: activated.");
  }

  @Override
  public void step() {
    tick++;
    if (reload > 0) reload--;

    // —— 0) 避障优先 —— //
    if (frontBlocked()) {
      mode = Mode.AVOID;
      if (avoidTicks == 0) {
        avoidRight = preferRight;
        preferRight = !preferRight; // 下次换边试试
      }
      avoidTicks = AVOID_STICK;
    } else if (mode == Mode.AVOID) {
      if (avoidTicks > 0) avoidTicks--;
      if (avoidTicks == 0) mode = Mode.SEARCH;
    }

    // —— 1) 感知与目标选择（非 AVOID 才进行） —— //
    if (mode != Mode.AVOID) {
      ArrayList<IRadarResult> radar = detectRadar();
      IRadarResult lock = pickTarget(radar);

      if (lock != null) {
        double dir = lock.getObjectDirection();
        // ★ 相对方位 -> 绝对角（若雷达已给绝对角，置开关为 true）
        targetDir = RADAR_BEARING_IS_ABSOLUTE ? normalizeAngle(dir)
                                              : normalizeAngle(getHeading() + dir);
        lastSeenDir = targetDir;
        lostTicks = 0;
        mode = Mode.ENGAGE;
      } else {
        lostTicks++;
        if (lostTicks > LOST_TIMEOUT) {
          mode = Mode.SEARCH;
          targetDir = Double.NaN;
        } else {
          targetDir = lastSeenDir; // 短暂丢失，维持朝向
          mode = Mode.ENGAGE;
        }
      }
    }

    // —— 2) 行为执行 —— //
    switch (mode) {
      case AVOID:
        avoidStep();
        break;
      case ENGAGE:
        engageStep();
        break;
      case SEARCH:
      default:
        searchStep();
        break;
    }
  }

  // ================== 行为：避障 ==================
  private void avoidStep() {
    // 转向 + 小撤退：快速脱离障碍法线
    stepTurn(avoidRight ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
    moveBack();
  }

  // ================== 行为：追击/交战 ==================
  private void engageStep() {
    double diff = signedAngleDiff(getHeading(), targetDir);

    // ★ 未对准：只转向，避免当 tick 同时转动又移动导致“转向被覆盖”
    if (Math.abs(diff) > AIM_TOL) {
      stepTurn(diff > 0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
      return; // 关键：早返回，当前 tick 不做移动
    }

    // 对准到宽容角：抖动扫射
    if (reload == 0 && Math.abs(diff) < FIRE_CONE) {
      double jitter = (Math.random() - 0.5) * AIM_TOL * 0.8; // -0.4~0.4*AIM_TOL
      fire(normalizeAngle(targetDir + jitter));
      reload = LOCAL_RELOAD;
    }

    // 轻量风筝：前后交替，减少贴脸互秒
    kitePhase = (kitePhase + 1) % KITE_PERIOD;
    if (kitePhase < KITE_PERIOD / 2) move(); else moveBack();
  }

  // ================== 行为：搜索 ==================
  private void searchStep() {
    // 弧线巡航 + 周期换边，降低贴墙概率
    if (tick % 35 == 0) preferRight = !preferRight;
    stepTurn(preferRight ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
    move();
    // 若想偶发试探射击，可打开下面两行：
    // if (reload == 0) { fire(getHeading()); reload = LOCAL_RELOAD; }
  }

  // ================== 传感与工具 ==================

  /** 统一选择目标：优先主机器人，其次副机器人 */
  private IRadarResult pickTarget(ArrayList<IRadarResult> list) {
    if (list == null || list.isEmpty()) return null;
    for (IRadarResult r : list) {
      if (r.getObjectType() == IRadarResult.Types.OpponentMainBot) return r;
    }
    for (IRadarResult r : list) {
      if (r.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) return r;
    }
    return null;
  }

  /**
   * 前方是否被障碍/实体占据（适配不同教学实现）：
   * - detectFront() 可能返回 IFrontSensorResult.Types（枚举）
   * - 或返回 IFrontSensorResult（对象，需要 getObjectType()）
   * - NOTHING/Nothing 均视为空
   */
  @SuppressWarnings("rawtypes")
  private boolean frontBlocked() {
    try {
      Object fr = detectFront();
      if (fr == null) return false;

      IFrontSensorResult.Types type = null;

      if (fr instanceof IFrontSensorResult) {
        type = ((IFrontSensorResult) fr).getObjectType();
      } else if (fr instanceof Enum) {
        type = (IFrontSensorResult.Types) fr;
      } else {
        return false; // 类型不识别，保守认为不阻塞
      }

      if (type == null) return false;

      String n = type.name(); // e.g., "Nothing", "NOTHING", "Wall", ...
      if (n.equalsIgnoreCase("Nothing")) return false;

      // 其余任何命中都视为阻塞（墙、友军、敌军、道具等）
      return true;

    } catch (Throwable t) {
      // 兼容性异常直接忽略：视为不阻塞
      return false;
    }
  }

  /** 返回 (target - current) 的最小有符号角度差，范围 (-PI, PI] */
  private static double signedAngleDiff(double current, double target) {
    double d = normalizeAngle(target - current);
    if (d > Math.PI)  d -= TWO_PI;
    if (d <= -Math.PI) d += TWO_PI;
    return d;
  }

  /** 归一化到 [0, 2π) */
  private static double normalizeAngle(double a) {
    a = a % TWO_PI;
    if (a < 0) a += TWO_PI;
    return a;
  }
}
