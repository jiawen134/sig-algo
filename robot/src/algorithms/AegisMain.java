package algorithms;

import java.util.*;
import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

public class AegisMain extends Brain {

  // ================= 常量与参数 =================
  private static final double TWO_PI = Math.PI * 2.0;

  // 一些“旋钮”
  private static final double TURN_HARD = 0.35;                 // 大角差(≈20°)时仅转头
  private static final double JITTER = 0.03 * Math.PI;          // 开火抖动(≈±5.4°)
  private static final int    LOCAL_RELOAD = Parameters.bulletFiringLatency; // 对齐≈20
  private static final int    LOST_TIMEOUT = 60;                // 丢失耐心
  private static final int    SECT = 64;                        // 扇区数(尸体/广播索引)
  private static final int    TTL_MSG = 60;                     // 消息有效期(tick)
  private static final int    TTL_CORPSE = 60;                  // 尸体扇区禁射期(tick)
  private static final double EPS_TEAM = 0.20;                  // 友军禁射锥(~11.5°)
  private static final double EPS_WRECK = 0.12;                 // 尸体禁射锥(~7°)

  // 自身物理（Team A 主机）
  private static final double MY_RANGE = Parameters.teamAMainBotFrontalDetectionRange; // 300
  private static final double MY_R     = Parameters.teamAMainBotRadius;                // 50
  private static final double VB       = Parameters.bulletVelocity;                    // 10mm/tick

  // 推进占优风筝：在20帧里 16前进/4后退
  private static final int KITE_PERIOD = 20;
  private static final int KITE_FWD    = 16;

  // ================= 运行态 =================
  private int tick = 0;
  private int reload = 0;
  private int lastFireTick = -9999;
  private int lostTicks = LOST_TIMEOUT + 1;

  private double lastAimAbs = Double.NaN;       // 最近一次看到的目标绝对角
  private double prevDirAbs = Double.NaN;       // 为角速度平滑
  private double angVel = 0.0;                  // 角速度平滑

  // 避障/巡航节拍
  private boolean avoidFlip = true;             // true:退一步 / false:转一步
  private boolean patrolBeat = false;           // 走/转轮换

  // 协同：广播限频与共享目标
  private final int[] lastBroadcastForSector = new int[SECT];

  private static class SharedTarget {
    double dir, dist, rad;  // 绝对角、距离、半径
    int lastSeen;           // 本地tick
    int sources;            // 贡献者数
    double trust;           // 简单信任度(0..1)
  }
  private final SharedTarget[] shared = new SharedTarget[SECT];

  // 尸体扇区禁射
  private final int[] corpseBanUntil = new int[SECT];

  // 身份（用于广播）
  private final String myId = "A_MAIN";

  // ================= 生命周期 =================
  @Override
  public void activate() {
    tick = 0; reload = 0; lastFireTick = -9999;
    lostTicks = LOST_TIMEOUT + 1;
    lastAimAbs = getHeading();
    prevDirAbs = Double.NaN; angVel = 0.0;
    Arrays.fill(lastBroadcastForSector, -9999);
    Arrays.fill(corpseBanUntil, 0);
    for (int i=0;i<SECT;i++) shared[i] = null;
    sendLogMessage("AegisMain: online (distance-aware).");
  }

  @Override
  public void step() {
    tick++; if (reload>0) reload--;

    // 处理消息 → 聚合共享目标
    processMessages();

    // 传感器
    IFrontSensorResult.Types frontT = frontType();
    ArrayList<IRadarResult> radar = detectRadar();

    // 选敌（主机更偏好敌方主机，其次距离近）
    IRadarResult local = pickTarget(radar, true);
    if (local != null) {
      double cur = normalize(local.getObjectDirection());   // ★绝对角
      if (!Double.isNaN(prevDirAbs)) {
        double d = signedDiff(prevDirAbs, cur);
        angVel = 0.6*angVel + 0.4*d;                        // 平滑角速度
      }
      prevDirAbs = cur;
      lastAimAbs = cur;
      lostTicks = 0;

      // 有稳定观测 → 尝试限频广播
      tryBroadcast(local);
    } else {
      lostTicks++;
    }

    // 若本地无目标，尝试用共享目标靠拢（不开火，先靠拢/对准）
    SharedTarget sharedBest = (local==null) ? selectBestShared() : null;

    // ====== 1) 避障：仅墙/友军 → 交替 退/转，确保离开法线 ======
    if (frontIsWallOrTeam(frontT)) {
      if (avoidFlip) moveBack(); else stepTurn(Parameters.Direction.RIGHT);
      avoidFlip = !avoidFlip;
      return;
    }

    // ====== 2) 射击窗口（冷却好 + 本地看到目标 + 炮口线清空） ======
    if (reload==0 && local!=null) {
      double aim = normalize(local.getObjectDirection());
      double d   = local.getObjectDistance();
      double r   = local.getObjectRadius();

      // 角速度+距离 的弹道前导（限幅）
      double dTheta = Math.atan2(angVel * d, VB);
      dTheta = clamp(dTheta, -0.35, +0.35);
      double lead = normalize(aim + dTheta);

      // 贴脸 → 先拉开
      if (d < MY_R + r + 30) { moveBack(); return; }

      // 炮口线清空检查（友伤/尸体/墙体）
      if (clearToFire(lead, d, r, radar, frontT)) {
        fire(lead); reload = LOCAL_RELOAD; lastFireTick = tick; return;  // ★ fire必须最后
      } else {
        // 被挡：短退一步，让出弹道
        moveBack(); return;
      }
    }

    // ====== 3) 有本地目标：大角差仅转头；小角差推进占优风筝 ======
    if (local != null) {
      double aim = normalize(local.getObjectDirection());
      double diff = signedDiff(getHeading(), aim);
      if (Math.abs(diff) > TURN_HARD) {
        stepTurn(diff>0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
        return;
      }
      // 推进占优风筝（产生净位移）
      int phase = tick % KITE_PERIOD;
      if (phase < KITE_FWD) move(); else moveBack();
      return;
    }

    // ====== 4) 仅有共享目标：对准/靠拢（不开火），直到本地雷达也看见 ======
    if (sharedBest != null) {
      double aim = normalize(sharedBest.dir);
      double diff = signedDiff(getHeading(), aim);
      // 先对准
      if (Math.abs(diff) > TURN_HARD) {
        stepTurn(diff>0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
        return;
      }
      // 再推进
      move(); 
      return;
    }

    // ====== 5) 无目标：巡航（走/转轮换，形成外扩弧线） ======
    patrolBeat = !patrolBeat;
    if (patrolBeat) move();
    else            stepTurn( ((tick/80)%2==0) ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT );
  }

  // ================= 传感/选择/清线 =================

  private IFrontSensorResult.Types frontType(){
    try {
      IFrontSensorResult r = detectFront();
      return (r==null) ? null : r.getObjectType();
    } catch(Throwable t){ return null; }
  }
  private static boolean frontIsWallOrTeam(IFrontSensorResult.Types t){
    if (t==null) return false;
    return t==IFrontSensorResult.Types.WALL
        || t==IFrontSensorResult.Types.TeamMainBot
        || t==IFrontSensorResult.Types.TeamSecondaryBot;
  }
  private static boolean isOpponent(IRadarResult.Types t){
    return t==IRadarResult.Types.OpponentMainBot || t==IRadarResult.Types.OpponentSecondaryBot;
  }
  private static boolean isTeam(IRadarResult.Types t){
    return t==IRadarResult.Types.TeamMainBot || t==IRadarResult.Types.TeamSecondaryBot;
  }

  /** 主机优先敌主机，其次距离近 */
  private IRadarResult pickTarget(ArrayList<IRadarResult> list, boolean preferMain){
    if (list==null || list.isEmpty()) return null;
    IRadarResult best=null; double bestScore=1e18;
    for (IRadarResult r: list){
      if (!isOpponent(r.getObjectType())) continue;
      double dist = r.getObjectDistance();
      double bias = (preferMain && r.getObjectType()==IRadarResult.Types.OpponentMainBot) ? 0.0 : 100.0;
      double score = dist + bias;
      if (score < bestScore){ bestScore=score; best=r; }
    }
    return best;
  }

  /** 炮口线“清线”：友军夹角禁射、尸体/残骸挡线禁射、正前WALL/Team禁射 */
  private boolean clearToFire(double aimAbs, double dTarget, double rTarget,
                              ArrayList<IRadarResult> radar, IFrontSensorResult.Types frontT)
  {
    // 正前方墙/友军直接禁射
    if (frontIsWallOrTeam(frontT)) return false;

    // 尸体扇区禁射TTL
    if (isCorpseSector(aimAbs)) return false;

    // 友军在同一射线附近且更近 → 禁射
    for (IRadarResult r: radar) {
      IRadarResult.Types t = r.getObjectType();
      if (isTeam(t)) {
        double da = Math.abs(signedDiff(aimAbs, normalize(r.getObjectDirection())));
        if (da < EPS_TEAM && r.getObjectDistance() < dTarget + rTarget) return false;
      }
    }
    // 残骸在弹道上且更近 → 禁射
    for (IRadarResult r: radar) {
      if (r.getObjectType() == IRadarResult.Types.Wreck) {
        double da = Math.abs(signedDiff(aimAbs, normalize(r.getObjectDirection())));
        if (da < EPS_WRECK && r.getObjectDistance() < Math.max(0, dTarget - r.getObjectRadius() - Parameters.bulletRadius))
          return false;
      }
    }
    return true;
  }

  // ================ 协同（广播/接收/聚合） ================
  private void tryBroadcast(IRadarResult r){
    // 只广播“活体”且在可战距离内
    if (!isOpponent(r.getObjectType())) return;
    double d = r.getObjectDistance();
    if (d <= 100 || d > MY_RANGE+1) return; // 太近/越界不播

    int k = sectorIndex(normalize(r.getObjectDirection()));
    if (tick - lastBroadcastForSector[k] < 8) return; // 限频：每扇区≥8tick

    lastBroadcastForSector[k] = tick;
    String msg = String.format(java.util.Locale.ROOT,
        "TGT|%s|%d|%.6f|%.1f|%.1f", myId, tick, r.getObjectDirection(), r.getObjectDistance(), r.getObjectRadius());
    broadcast(msg);
  }

  private void processMessages(){
    ArrayList<String> msgs = fetchAllMessages();
    if (msgs==null || msgs.isEmpty()) return;
    for (String m: msgs){
      try{
        String[] p = m.split("\\|");
        if (p.length<6 || !"TGT".equals(p[0])) continue;
        int msgTick = Integer.parseInt(p[2]);
        if (tick - msgTick > TTL_MSG) continue;

        double dir = Double.parseDouble(p[3]);
        double dist= Double.parseDouble(p[4]);
        double rad = Double.parseDouble(p[5]);
        int k = sectorIndex(normalize(dir));

        SharedTarget t = shared[k];
        if (t==null) { t = new SharedTarget(); shared[k]=t; }

        double wOld = Math.max(0, t.sources);
        t.dir = (wOld==0)? dir : (t.dir*wOld + dir)/(wOld+1);
        t.dist= (wOld==0)? dist: (t.dist*wOld+ dist)/(wOld+1);
        t.rad = (wOld==0)? rad : Math.max(t.rad, rad);
        t.lastSeen = tick;
        t.sources = (int)(wOld+1);
        t.trust = Math.min(1.0, t.sources/3.0); // 多源提高信任
      } catch(Throwable ignore){}
    }
  }

  private SharedTarget selectBestShared(){
    SharedTarget best=null; double bestScore=1e18;
    for (int i=0;i<SECT;i++){
      SharedTarget t = shared[i];
      if (t==null) continue;
      if (tick - t.lastSeen > TTL_MSG) { shared[i]=null; continue; }
      // 简单打分：信任高 + 距离近
      double score = t.dist - 100*t.trust;
      if (score < bestScore){ bestScore=score; best=t; }
    }
    return best;
  }

  // ================ 尸体扇区禁射工具 ================
  private void markCorpseAt(double angle){ corpseBanUntil[sectorIndex(angle)] = Math.max(corpseBanUntil[sectorIndex(angle)], tick + TTL_CORPSE); }
  private boolean isCorpseSector(double angle){ return corpseBanUntil[sectorIndex(angle)] > tick; }

  private int sectorIndex(double angle){
    double a = normalize(angle);
    int k = (int)Math.floor(a / TWO_PI * SECT);
    if (k<0) k=0; if (k>=SECT) k=SECT-1; return k;
  }

  // ================ 角度/数学工具 ================
  private static double normalize(double a){ a%=TWO_PI; if (a<0) a+=TWO_PI; return a; }
  private static double signedDiff(double from, double to){
    double d = normalize(to - from);
    if (d> Math.PI) d -= TWO_PI;
    if (d<=-Math.PI) d += TWO_PI;
    return d;
  }
  private static double clamp(double x, double lo, double hi){ return (x<lo)?lo : (x>hi)?hi : x; }
}
