package algorithms;

import java.util.*;
import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

public class AegisMain extends Brain {

  // ===== 常量 =====
  private static final double TWO_PI = Math.PI * 2.0;
  private static final double JITTER = 0.02 * Math.PI;          // 开火轻微抖动，减少被边缘挡住
  private static final int    LOCAL_RELOAD = Parameters.bulletFiringLatency; // ~20
  private static final int    POS_PERIOD   = 25;                // 广播自身坐标周期（也便于队友清线）
  private static final int    TXY_LIMIT    = 8;                 // 每扇区TXY限频
  private static final int    TTL_MSG      = 80;                // POS/TXY有效期
  private static final int    SECT         = 64;
  private static final double BULLET_RANGE = Parameters.bulletRange;        // 1000
  private static final double BULLET_R     = Parameters.bulletRadius;       // 5
  private static final double STEP         = Parameters.teamAMainBotSpeed;  // 1
  private static final double VB           = Parameters.bulletVelocity;     // 10

  // 走廊锁：清线一次后短时间专注射击（不再重复清线）
  private static final int    LOCK_TTL_TICKS = 30;
  private static final double LOCK_ANG_EPS   = 0.10;
  private static final double LOCK_DIST_EPS  = 60.0;

  // ===== 运行态 =====
  private int tick=0, reload=0, lastPosBroadcast=-9999;
  private final String myId = "A_MAIN_" + Integer.toHexString((int)(Math.random()*0xFFFF));

  // 里程计（世界坐标）
  private double posX, posY, odoPend=0.0, odoHdg=0.0;

  // 敌/友世界坐标
  private static class EnemyXY { double x,y,rad; int last; }
  private static class AllyPos { double x,y,hdg; int last; String id; }
  private final List<EnemyXY> enemyXY = new ArrayList<>();
  private final Map<String, AllyPos> ally = new HashMap<>();
  private final int[] lastTxyBroadcast = new int[SECT];

  // Alpha–Beta 常速轨迹
  private static class ABTrack {
    double x,y,vx,vy,rad; int last;
    void init(double X,double Y,double R,int t){x=X;y=Y;vx=0;vy=0;rad=R;last=t;}
    void update(double mx,double my,int t){
      double xp=x+vx, yp=y+vy;
      double rx=mx-xp, ry=my-yp;
      double a=0.6, b=0.2; // α,β
      x = xp + a*rx;  y = yp + a*ry;
      vx = vx + b*rx; vy = vy + b*ry;
      last = t;
    }
    ABTrack copy(){ ABTrack z=new ABTrack(); z.x=x; z.y=y; z.vx=vx; z.vy=vy; z.rad=rad; z.last=last; return z; }
    void lead(double tf){ x += vx*tf; y += vy*tf; }
  }
  private final Map<Integer, ABTrack> tracks = new HashMap<>();
  private static int keyOf(double x,double y){ int gx=(int)Math.floor(x/50.0), gy=(int)Math.floor(y/50.0); return (gx<<16) ^ (gy & 0xFFFF); }

  // 走廊锁
  private boolean fireLock=false; private double lockAim=0, lockDist=0; private int lockTick=-9999;

  @Override
  public void activate() {
    tick=0; reload=0; Arrays.fill(lastTxyBroadcast, -9999);
    posX = Parameters.teamAMainBot1InitX;
    posY = Parameters.teamAMainBot1InitY;
    odoPend=0.0; odoHdg=getHeading();
    fireLock=false;
    sendLogMessage("AegisMain: rule-only (AB lead + corridor lock + intel long-shot).");
  }

  @Override
  public void step() {
    tick++; if (reload>0) reload--;
    applyOdo();

    // 周期广播自身坐标（供队友清线/避让）
    if (tick - lastPosBroadcast >= POS_PERIOD){
      lastPosBroadcast = tick;
      String msg = String.format(java.util.Locale.ROOT, "POS|%s|%d|%.1f|%.1f|%.6f", myId, tick, posX, posY, getHeading());
      broadcast(msg);
    }

    processMessages();
    ingestObservations(); // TXY + 雷达 → 轨迹集

    // 选择进入射程的最近目标（AB按飞行时间前导）
    Target tgt = pickShootableTarget();

    if (tgt != null) {
      // 走廊锁有效：直接开火
      if (reload==0 && fireLockValid(tgt.aim, tgt.dist)) {
        fire(withJitter(tgt.aim)); reload=LOCAL_RELOAD; lockTick=tick; return;
      }
      // 几何清线（友军/尸体）通过 → 开火并上锁
      if (reload==0 && clearByGeometry(tgt.aim, tgt.dist, tgt.rad)) {
        fire(withJitter(tgt.aim)); reload=LOCAL_RELOAD; armLock(tgt.aim, tgt.dist); return;
      }
      // 被挡：不移动，等走廊变干净（专注射击）
      return;
    }

    // 无可射目标：直线向东轻推（不摇头），遇阻右转
    IFrontSensorResult.Types ft = frontType();
    if (frontIsBlock(ft)) { stepTurn(Parameters.Direction.RIGHT); return; }
    double diffE = signedDiff(getHeading(), Parameters.EAST);
    if (Math.abs(diffE) > 0.30) stepTurn(diffE>0?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
    else odoMove();
  }

  // ===== 观测/目标 =====
  private static class Target { double aim, dist, rad; ABTrack src; }

  private void ingestObservations(){
    // a) TXY 敌情
    for (EnemyXY e : enemyXY) {
      if (tick - e.last > TTL_MSG) continue;
      updateTrack(e.x, e.y, e.rad);
    }
    // b) 本地雷达转世界坐标（并转发TXY，方便其他主机远射）
    ArrayList<IRadarResult> rs = detectRadar();
    if (rs!=null) for (IRadarResult r: rs){
      if (r.getObjectType()==IRadarResult.Types.OpponentMainBot ||
          r.getObjectType()==IRadarResult.Types.OpponentSecondaryBot){
        double a=r.getObjectDirection(), d=r.getObjectDistance(), rad=r.getObjectRadius();
        double ex = posX + d*Math.cos(a), ey = posY + d*Math.sin(a);
        updateTrack(ex, ey, rad);
        tryBroadcastTXY(a, d, rad);
      }
    }
    // c) 清理过期
    tracks.entrySet().removeIf(en -> tick - en.getValue().last > TTL_MSG);
  }

  private void updateTrack(double ex,double ey,double rad){
    int k = keyOf(ex, ey);
    ABTrack t = tracks.get(k);
    if (t==null){
      // 在邻格找最近轨迹
      ABTrack near=null; double best=1e18;
      for (int dx=-1; dx<=1; dx++) for (int dy=-1; dy<=1; dy++){
        ABTrack c = tracks.get(k + ((dx)<<16) ^ (dy & 0xFFFF));
        if (c==null) continue;
        double dd = hypot(c.x-ex, c.y-ey);
        if (dd<best){ best=dd; near=c; }
      }
      if (near!=null && best<150.0) t=near;
      else { t=new ABTrack(); t.init(ex,ey,rad,tick); tracks.put(k,t); }
    }
    t.rad=rad; t.update(ex,ey,tick);
  }

  private Target pickShootableTarget(){
    Target best=null; double bestD=1e18;
    for (ABTrack tr : tracks.values()){
      ABTrack tmp = tr.copy();
      double distNow = hypot(tmp.x - posX, tmp.y - posY);
      double tf = distNow / VB;
      tmp.lead(tf);
      double px=tmp.x, py=tmp.y;
      double aim = Math.atan2(py - posY, px - posX);
      double d   = hypot(px - posX, py - posY);
      if (d<=BULLET_RANGE && d<bestD){
        bestD=d; best=new Target(); best.aim=aim; best.dist=d; best.rad=tr.rad; best.src=tr;
      }
    }
    return best;
  }

  // ===== 几何清线（友军 POS + 残骸 Radar→世界坐标） =====
  private boolean clearByGeometry(double aimAbs, double dTarget, double rTarget){
    // 友军挡线
    for (AllyPos ap : ally.values()){
      if (tick - ap.last > TTL_MSG) continue;
      if (blocksLine(ap.x, ap.y, 50, dTarget, aimAbs)) return false;
    }
    // 残骸挡线
    ArrayList<IRadarResult> rs = detectRadar();
    if (rs!=null) for (IRadarResult r: rs){
      if (r.getObjectType()==IRadarResult.Types.Wreck){
        double wx = posX + r.getObjectDistance()*Math.cos(r.getObjectDirection());
        double wy = posY + r.getObjectDistance()*Math.sin(r.getObjectDirection());
        if (blocksLine(wx, wy, r.getObjectRadius(), dTarget, aimAbs)) return false;
      }
    }
    return true;
  }

  // 线段(我→目标) 与 圆(ox,oy,rad) 是否相交（且在目标之前）
  private boolean blocksLine(double ox,double oy,double rad,double dTarget,double aimAbs){
    double dx = ox - posX, dy = oy - posY;
    double ux = Math.cos(aimAbs), uy = Math.sin(aimAbs);
    double u = (dx*ux + dy*uy) / dTarget;
    if (u <= 0 || u >= 1) return false;
    double px = u*dTarget*ux, py = u*dTarget*uy;
    double perp = hypot(dx - px, dy - py);
    return (perp <= rad + BULLET_R + 6);
  }

  // ===== 走廊锁 =====
  private boolean fireLockValid(double aimAbs, double dTarget){
    if (!fireLock) return false;
    if (tick - lockTick > LOCK_TTL_TICKS) return false;
    if (Math.abs(signedDiff(lockAim, aimAbs)) > LOCK_ANG_EPS) return false;
    if (Math.abs(lockDist - dTarget) > LOCK_DIST_EPS) return false;
    return true;
  }
  private void armLock(double aimAbs,double dTarget){ fireLock=true; lockAim=aimAbs; lockDist=dTarget; lockTick=tick; }

  // ===== 消息 =====
  private void processMessages(){
    ArrayList<String> msgs = fetchAllMessages();
    if (msgs!=null) for (String m: msgs){
      try{
        String[] p = m.split("\\|");
        if (p.length<1) continue;
        if ("POS".equals(p[0]) && p.length>=7){
          String id=p[1]; double x=Double.parseDouble(p[3]), y=Double.parseDouble(p[4]), hdg=Double.parseDouble(p[5]);
          AllyPos ap = ally.getOrDefault(id, new AllyPos());
          ap.x=x; ap.y=y; ap.hdg=hdg; ap.last=tick; ap.id=id; ally.put(id, ap);
        } else if ("TXY".equals(p[0]) && p.length>=6){
          double x=Double.parseDouble(p[3]), y=Double.parseDouble(p[4]), rad=Double.parseDouble(p[5]);
          EnemyXY e = new EnemyXY(); e.x=x; e.y=y; e.rad=rad; e.last=tick; enemyXY.add(e);
        }
      }catch(Throwable ignore){}
    }
    ally.entrySet().removeIf(en -> tick - en.getValue().last > TTL_MSG);
    enemyXY.removeIf(e -> tick - e.last > TTL_MSG);
  }

  // 将本地雷达的极坐标转换为世界坐标并广播（便于其他主机远射）
  private void tryBroadcastTXY(double dirAbs,double dist,double rad){
    int k = sectorIndex(dirAbs);
    if (tick - lastTxyBroadcast[k] < TXY_LIMIT) return;
    lastTxyBroadcast[k]=tick;
    double ex = posX + dist*Math.cos(dirAbs);
    double ey = posY + dist*Math.sin(dirAbs);
    String msg = String.format(java.util.Locale.ROOT, "TXY|%s|%d|%.1f|%.1f|%.1f", myId, tick, ex, ey, rad);
    broadcast(msg);
  }

  // ===== 里程计 / 工具 =====
  private void applyOdo(){ if (Math.abs(odoPend)>1e-9){ posX += odoPend*Math.cos(odoHdg); posY += odoPend*Math.sin(odoHdg); odoPend=0.0; } }
  private void odoMove(){ odoPend += STEP; odoHdg=getHeading(); move(); }

  private IFrontSensorResult.Types frontType(){ try{ IFrontSensorResult r=detectFront(); return (r==null)?null:r.getObjectType(); }catch(Throwable t){return null;} }
  private boolean frontIsBlock(IFrontSensorResult.Types t){
    if (t==null) return false;
    return t==IFrontSensorResult.Types.WALL || t==IFrontSensorResult.Types.Wreck
        || t==IFrontSensorResult.Types.TeamMainBot || t==IFrontSensorResult.Types.TeamSecondaryBot;
  }

  private int sectorIndex(double a){ double x=normalize(a); int k=(int)Math.floor(x/TWO_PI*SECT); if(k<0)k=0; if(k>=SECT)k=SECT-1; return k; }
  private static double normalize(double a){ a%=TWO_PI; if (a<0) a+=TWO_PI; return a; }
  private static double signedDiff(double from,double to){ double d=normalize(to-from); if(d>Math.PI)d-=TWO_PI; if(d<=-Math.PI)d+=TWO_PI; return d; }
  private static double withJitter(double a){ return normalize(a + (Math.random()-0.5)*JITTER*2.0); }
  private static double hypot(double x,double y){ return Math.sqrt(x*x + y*y); }
}
