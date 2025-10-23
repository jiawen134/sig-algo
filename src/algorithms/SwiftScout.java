package algorithms;

import java.util.*;
import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

public class SwiftScout extends Brain {

  // =============== 常量与参数 ===============
  private static final double TWO_PI = Math.PI * 2.0;

  private static final double TURN_HARD = 0.30;                 // 小些，更灵活(≈17°)
  private static final double JITTER    = 0.03*Math.PI;
  private static final int    LOCAL_RELOAD = Math.max(8, Parameters.bulletFiringLatency-2); // 略快尝试
  private static final int    LOST_TIMEOUT = 40;

  private static final int    SECT = 64;
  private static final int    TTL_MSG = 60;
  private static final int    TTL_CORPSE = 60;
  private static final double EPS_TEAM  = 0.20;
  private static final double EPS_WRECK = 0.12;

  // 自身物理（Team A 副机）
  private static final double MY_RANGE = Parameters.teamASecondaryBotFrontalDetectionRange; // 500
  private static final double MY_R     = Parameters.teamASecondaryBotRadius;                // 50
  private static final double VB       = Parameters.bulletVelocity;                         // 10

  // 外扩弧线：三拍节奏 2走1转
  private int beat = 0;
  private boolean orbitRight = true;
  private boolean avoidFlip = true;

  // 运行态
  private int tick=0, reload=0, lastFireTick=-9999, lostTicks=LOST_TIMEOUT+1;
  private double prevDirAbs = Double.NaN, angVel=0.0;

  // 协同
  private final int[] lastBroadcastForSector = new int[SECT];
  private final SharedTarget[] shared = new SharedTarget[SECT];
  private final int[] corpseBanUntil = new int[SECT];
  private final String myId = "A_SCOUT";

  private static class SharedTarget {
    double dir, dist, rad; int lastSeen, sources; double trust;
  }

  @Override
  public void activate() {
    tick=0; reload=0; lastFireTick=-9999; lostTicks=LOST_TIMEOUT+1;
    prevDirAbs=Double.NaN; angVel=0.0; beat=0; orbitRight=true; avoidFlip=true;
    Arrays.fill(lastBroadcastForSector, -9999);
    Arrays.fill(corpseBanUntil, 0);
    for (int i=0;i<SECT;i++) shared[i]=null;
    sendLogMessage("SwiftScout: online (distance-aware).");
  }

  @Override
  public void step() {
    tick++; if (reload>0) reload--;

    processMessages();
    IFrontSensorResult.Types frontT = frontType();
    ArrayList<IRadarResult> radar = detectRadar();

    // 副机：最近优先（灵活骚扰）
    IRadarResult local = pickNearestOpponent(radar);
    if (local!=null){
      double cur = normalize(local.getObjectDirection());
      if (!Double.isNaN(prevDirAbs)){
        double d = signedDiff(prevDirAbs, cur);
        angVel = 0.6*angVel + 0.4*d;
      }
      prevDirAbs = cur; lostTicks=0;
      tryBroadcast(local);
    } else {
      lostTicks++;
    }

    SharedTarget sharedBest = (local==null) ? selectBestShared() : null;

    // 1) 避障：仅墙/友军 → 交替 退/转
    if (frontIsWallOrTeam(frontT)) {
      if (avoidFlip) moveBack(); else stepTurn(orbitRight?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
      avoidFlip=!avoidFlip;
      return;
    }

    // 2) 射击窗口（冷却好 + 本地看到 + 清线）
    if (reload==0 && local!=null){
      double aim = normalize(local.getObjectDirection());
      double d   = local.getObjectDistance();
      double r   = local.getObjectRadius();

      double dTheta = Math.atan2(angVel * d, VB);
      dTheta = clamp(dTheta, -0.35, +0.35);
      double lead = normalize(aim + dTheta);

      if (d < MY_R + r + 30) { moveBack(); return; }

      if (clearToFire(lead, d, r, radar, frontT)){
        fire(lead); reload=LOCAL_RELOAD; lastFireTick=tick; return;
      } else {
        // 给出炮位：绕侧一步
        stepTurn(orbitRight?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
        return;
      }
    }

    // 3) 有本地目标：小角差推进，2走1转外扩
    if (local!=null){
      double aim = normalize(local.getObjectDirection());
      double diff = signedDiff(getHeading(), aim);
      if (Math.abs(diff) > TURN_HARD){
        stepTurn(diff>0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
        return;
      }
      beat = (beat+1)%3;
      if (beat<2) move(); else stepTurn(orbitRight?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
      // 偶发翻向，避免局部小圈
      if (tick%180==0 || lostTicks>LOST_TIMEOUT/2) orbitRight=!orbitRight;
      return;
    }

    // 4) 只有共享目标：先对准再推进（不开火，直到本地看到）
    if (sharedBest!=null){
      double aim = normalize(sharedBest.dir);
      double diff = signedDiff(getHeading(), aim);
      if (Math.abs(diff) > TURN_HARD){
        stepTurn(diff>0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
        return;
      }
      beat = (beat+1)%3;
      if (beat<2) move(); else stepTurn(orbitRight?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
      return;
    }

    // 5) 无目标：扫场(2走1转)
    beat = (beat+1)%3;
    if (beat<2) move(); else stepTurn(orbitRight?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
  }

  // =================== 传感/工具 ===================
  private IFrontSensorResult.Types frontType(){
    try { IFrontSensorResult r = detectFront(); return (r==null)? null : r.getObjectType(); }
    catch(Throwable t){ return null; }
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
  private IRadarResult pickNearestOpponent(ArrayList<IRadarResult> list){
    if (list==null || list.isEmpty()) return null;
    IRadarResult best=null; double bestD=1e18;
    for (IRadarResult r: list){
      if (!isOpponent(r.getObjectType())) continue;
      double d = r.getObjectDistance();
      if (d < bestD){ bestD=d; best=r; }
    }
    return best;
  }

  private boolean clearToFire(double aimAbs, double dTarget, double rTarget,
                              ArrayList<IRadarResult> radar, IFrontSensorResult.Types frontT)
  {
    if (frontIsWallOrTeam(frontT)) return false;
    if (isCorpseSector(aimAbs)) return false;

    for (IRadarResult r: radar) {
      if (r.getObjectType()==IRadarResult.Types.TeamMainBot || r.getObjectType()==IRadarResult.Types.TeamSecondaryBot){
        double da = Math.abs(signedDiff(aimAbs, normalize(r.getObjectDirection())));
        if (da < EPS_TEAM && r.getObjectDistance() < dTarget + rTarget) return false;
      }
    }
    for (IRadarResult r: radar) {
      if (r.getObjectType()==IRadarResult.Types.Wreck){
        double da = Math.abs(signedDiff(aimAbs, normalize(r.getObjectDirection())));
        if (da < EPS_WRECK && r.getObjectDistance() < Math.max(0, dTarget - r.getObjectRadius() - Parameters.bulletRadius))
          return false;
      }
    }
    return true;
  }

  // ============== 协同（广播/接收/聚合） ==============
  private void tryBroadcast(IRadarResult r){
    if (!isOpponent(r.getObjectType())) return;
    double d = r.getObjectDistance();
    if (d <= 120 || d > MY_RANGE+1) return; // 太近/越界不播
    int k = sectorIndex(normalize(r.getObjectDirection()));
    if (tick - lastBroadcastForSector[k] < 8) return;
    lastBroadcastForSector[k]=tick;
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
        if (t==null){ t=new SharedTarget(); shared[k]=t; }
        double wOld = Math.max(0, t.sources);
        t.dir = (wOld==0)?dir : (t.dir*wOld+dir)/(wOld+1);
        t.dist= (wOld==0)?dist: (t.dist*wOld+dist)/(wOld+1);
        t.rad = (wOld==0)?rad : Math.max(t.rad, rad);
        t.lastSeen = tick; t.sources=(int)(wOld+1);
        t.trust = Math.min(1.0, t.sources/3.0);
      }catch(Throwable ignore){}
    }
  }

  private SharedTarget selectBestShared(){
    SharedTarget best=null; double bestScore=1e18;
    for (int i=0;i<SECT;i++){
      SharedTarget t = shared[i];
      if (t==null) continue;
      if (tick - t.lastSeen > TTL_MSG) { shared[i]=null; continue; }
      double score = t.dist - 100*t.trust;
      if (score < bestScore){ bestScore=score; best=t; }
    }
    return best;
  }

  // ============== 尸体扇区禁射 ==============
  private void markCorpseAt(double angle){ corpseBanUntil[sectorIndex(angle)] = Math.max(corpseBanUntil[sectorIndex(angle)], tick + TTL_CORPSE); }
  private boolean isCorpseSector(double angle){ return corpseBanUntil[sectorIndex(angle)] > tick; }
  private int sectorIndex(double angle){
    double a = normalize(angle);
    int k = (int)Math.floor(a / TWO_PI * SECT);
    if (k<0) k=0; if (k>=SECT) k=SECT-1; return k;
  }

  // ============== 角度工具 ==============
  private static double normalize(double a){ a%=TWO_PI; if (a<0) a+=TWO_PI; return a; }
  private static double signedDiff(double from, double to){
    double d = normalize(to - from);
    if (d> Math.PI) d -= TWO_PI;
    if (d<=-Math.PI) d += TWO_PI;
    return d;
  }
  private static double clamp(double x, double lo, double hi){ return (x<lo)?lo : (x>hi)?hi : x; }
}
