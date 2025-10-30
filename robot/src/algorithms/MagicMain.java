package algorithms;

import java.util.*;
import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

public class MagicMain extends Brain {
	
  // ===== 常量 =====
  private static final double TWO_PI = Math.PI * 2.0;
  private static final double BR  = Parameters.bulletRange;       // 1000
  private static final double BUL = Parameters.bulletRadius;      // 5
  private static final double STEP= Parameters.teamAMainBotSpeed; // 1

  private static final int SECT = 64, POS_PERIOD = 25, TXY_LIMIT = 8, TTL_MSG = 80;

  // 清线余量
  private static final double SAFE_PAD = 6.0;

  // 走廊锁（免清线窗口）——严格：需要"近期观测"才生效
  private static final int    LOCK_TTL_TICKS = 18;     // < 21 硬冷却
  private static final double LOCK_ANG_EPS   = 0.10;
  private static final double LOCK_DIST_EPS  = 70.0;
  private static final int    FRESH_MAX_AGE  = 20;     // 目标"新鲜度"门槛（tick）
  private static final int    LOCAL_MAX_AGE  = 5;      // 认为"雷达可信"的年龄

  // 避障（墙/友军/残骸）粘性
  private static final int    AVOID_TICKS = 15;
  private static final int    AVOID_BACK_TICKS = 3;  // 只后退前3个tick
  private boolean preferRight=true, avoidRight=true;
  private int avoidLeft=0;

  // ===== 运行态 =====
  private int tick=0, lastPosBroadcast=-9999;
  private final String myId = "A_MAIN_" + Integer.toHexString((int)(Math.random()*0xFFFF));

  // 里程计
  private double posX, posY, odoPend=0.0, odoHdg=0.0;

  // 启动自定位
  private boolean calibrated=false;

  // 友军/敌情缓存
  private static class AllyPos { double x,y,hdg; int last; String id; }
  private final Map<String, AllyPos> ally = new HashMap<>();
  private static class EnemyXY { double x,y,rad; int last; }
  private final List<EnemyXY> enemyXY = new ArrayList<>();
  private final int[] lastTxyBroadcast = new int[SECT];

  // 轨迹（常速）
  private static class Track {
    double x,y,vx,vy,rad; int last, lastLocal, seen;
    void init(double X,double Y,double R,int t){x=X;y=Y;vx=0;vy=0;rad=R;last=t;lastLocal=-9999;seen=0;}
    void update(double mx,double my,int t, boolean local){
      double xp=x+vx, yp=y+vy, rx=mx-xp, ry=my-yp; double a=0.6,b=0.2;
      x=xp+a*rx; y=yp+a*ry; vx=vx+b*rx; vy=vy+b*ry; last=t; seen++;
      if (local) lastLocal=t;
    }
  }
  private final List<Track> tracks = new ArrayList<>();

  // 走廊锁
  private boolean fireLock=false; private double lockAim=0, lockDist=0; private int lockUntil=-1, lockLastObs=-1;

  // ===== 生命周期 =====
  @Override public void activate() {
    tick=0; Arrays.fill(lastTxyBroadcast,-9999);
    // 先放默认坐标，等自定位
    posX = Parameters.teamAMainBot1InitX;
    posY = Parameters.teamAMainBot1InitY;
    calibrated=false;
    odoPend=0.0; odoHdg=getHeading();
    sendLogMessage("MagicMain: direct fire + sticky avoidance + safe lock.");
  }

  @Override public void step() {
    tick++; applyOdo();

    // —— 粘性避障优先 —— //
    IFrontSensorResult.Types ftNow = frontType();
    if (isObstacle(ftNow)) {
      if (avoidLeft==0) { 
        avoidRight=preferRight; 
        preferRight=!preferRight; 
        avoidLeft = AVOID_TICKS;
      }
    }
    if (avoidLeft>0) {
      int remaining = avoidLeft;
      stepTurn(avoidRight?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
      
      // 只在前几个tick后退，避免无限后退循环
      if (remaining > (AVOID_TICKS - AVOID_BACK_TICKS)) {
        moveBack();
      } else {
        // 后面的tick只转向，不后退
        // 不移动，让转向生效
      }
      
      avoidLeft--;
      return;
    }
    
    // —— 自定位：识别 顶/中/底 —— //
    if (!calibrated) {
      if (!calibrateByRadar()) {
        // 未定位期间小巡航避免卡死
        if (ftNow==IFrontSensorResult.Types.WALL) stepTurn(Parameters.Direction.RIGHT);
        else odoMove();
			return;
		} 
	}
	
    // 周期广播自身坐标
    if (tick - lastPosBroadcast >= POS_PERIOD){
      lastPosBroadcast=tick;
      broadcast(String.format(java.util.Locale.ROOT,"POS|%s|%d|%.1f|%.1f|%.6f",myId,tick,posX,posY,getHeading()));
    }

    // 消息融合
    processMessages();

    // 本地雷达→世界坐标 + TXY 转播
    ArrayList<IRadarResult> rs = detectRadar();
    if (rs!=null) for (IRadarResult r: rs){
      if (r.getObjectType()==IRadarResult.Types.OpponentMainBot ||
          r.getObjectType()==IRadarResult.Types.OpponentSecondaryBot){
        double a=r.getObjectDirection(), d=r.getObjectDistance(), rad=r.getObjectRadius();
        double ex=posX + d*Math.cos(a), ey=posY + d*Math.sin(a);
        updateTracks(ex,ey,rad,true);
        tryBroadcastTXY(a,d,rad);
      }
    }
    // 清理过期
    tracks.removeIf(t -> tick - t.last > TTL_MSG);

    // 求解射击
    AimSolution sol = pickBestSolution();
    if (sol != null) {
      // a) 锁有效且最近一次观测不陈旧 → 每 tick 射，交给引擎限速（不会乱开枪）
      if (fireLockValid(sol.aim, sol.dist)) {
        fire(sol.aim);
        return;
      }
      // b) 否则做一次几何清线，通过→开火并上锁；不通过→暂缓
      if (clearToFire(sol.aim, sol.dist)) {
        fire(sol.aim);
        armLock(sol.aim, sol.dist);
        return;
      }
      // 清线不通过：轻微移动调整位置，避免卡死
      // 每2个tick移动一次，其余时间转向
      if (tick%3 == 0) {
        odoMove();
      } else {
        stepTurn(preferRight?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
      }
      if (tick%50==0) preferRight=!preferRight;
      return;
    }

    // 无可射解：小步直线巡航 + 墙面避障
    if (ftNow==IFrontSensorResult.Types.WALL) { stepTurn(Parameters.Direction.RIGHT); return; }
    double dE = angDiff(getHeading(), Parameters.EAST);
    if (Math.abs(dE) > 0.30) stepTurn(dE>0?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
    else odoMove();
  }

  // ===== 自定位（顶/中/底） =====
  private boolean calibrateByRadar(){
    ArrayList<IRadarResult> rs = detectRadar();
    if (rs==null) return false;
    int north=0, south=0;
    for (IRadarResult r: rs){
      if (r.getObjectType()==IRadarResult.Types.TeamMainBot){
        if (Math.sin(r.getObjectDirection()) >= 0) south++; else north++;
      }
    }
    if (north+south==0) return false;
    posX = Parameters.teamAMainBot1InitX; // A 队三台主机 X 相同
    if (north>0 && south>0)      posY = Parameters.teamAMainBot2InitY; // 中
    else if (south>0)            posY = Parameters.teamAMainBot1InitY; // 顶
    else                         posY = Parameters.teamAMainBot3InitY; // 底
    calibrated = true;
    return true;
  }

  // ===== 射击解（直射，不预测移动） =====
  private static class AimSolution { double aim, dist; int freshness, localAge; }
  private AimSolution pickBestSolution(){
    AimSolution best=null; double bestScore=1e18;
    for (Track tr: tracks){
      int fresh = tick - tr.last;
      if (fresh > FRESH_MAX_AGE) continue;                 // 过陈旧不打
      int lAge  = (tr.lastLocal<0)? 9999 : tick - tr.lastLocal;

      // 直射（不预测敌人移动）
      double dx=tr.x-posX, dy=tr.y-posY;
      double aim  = Math.atan2(dy, dx);
      double dist = Math.hypot(dx, dy);
      
      if (dist>BR) continue;

      // 打分：距离近 + 最近本机雷达优先
      double score = dist + (lAge<=LOCAL_MAX_AGE? 0 : 200) + fresh*2;
      if (score < bestScore){
        bestScore=score;
        best=new AimSolution(); best.aim=aim; best.dist=dist; best.freshness=fresh; best.localAge=lAge;
      }
    }
    return best;
  }

  // ===== 走廊锁（基于近期观测，杜绝乱开枪） =====
  private boolean fireLockValid(double aimAbs,double dTarget){
    if (!fireLock) return false;
    if (tick >= lockUntil) return false;
    if (tick - lockLastObs > FRESH_MAX_AGE) return false;  // 观测陈旧 -> 失效
    if (Math.abs(angDiff(lockAim, aimAbs)) > LOCK_ANG_EPS) return false;
    if (Math.abs(lockDist - dTarget) > LOCK_DIST_EPS) return false;
    return true;
  }
  private void armLock(double aimAbs,double dTarget){
    fireLock=true; lockAim=aimAbs; lockDist=dTarget; lockUntil=tick + LOCK_TTL_TICKS; lockLastObs=tick;
  }

  // ===== 几何清线（友/残骸） + 前方仅“墙”硬拦 =====
  private boolean clearToFire(double aimAbs, double dTarget){
    if (frontType()==IFrontSensorResult.Types.WALL) return false; // 避免贴墙自爆

    // 友军
    for (AllyPos ap : ally.values()){
      if (tick - ap.last > TTL_MSG) continue;
      if (blocksLine(ap.x, ap.y, 50.0, dTarget, aimAbs)) return false;
    }
    // 残骸
    ArrayList<IRadarResult> rs = detectRadar();
    if (rs!=null) for (IRadarResult r: rs){
      if (r.getObjectType()==IRadarResult.Types.Wreck){
        double wx=posX + r.getObjectDistance()*Math.cos(r.getObjectDirection());
        double wy=posY + r.getObjectDistance()*Math.sin(r.getObjectDirection());
        if (blocksLine(wx, wy, r.getObjectRadius(), dTarget, aimAbs)) return false;
      }
    }
    return true;
  }
  private boolean blocksLine(double ox,double oy,double rad,double dTarget,double aimAbs){
    double ux=Math.cos(aimAbs), uy=Math.sin(aimAbs);
    double dx=ox-posX, dy=oy-posY;
    double u=(dx*ux+dy*uy)/dTarget; if (u<=0 || u>=1) return false;
    double px=u*dTarget*ux, py=u*dTarget*uy;
    double perp=Math.hypot(dx-px, dy-py);
    return perp <= (rad + BUL + SAFE_PAD);
  }

  // ===== 敌情/友军融合 + TXY 广播 =====
  private void processMessages(){
    ArrayList<String> msgs = fetchAllMessages();
    if (msgs==null) return;
    for (String m: msgs){
      try{
        String[] p=m.split("\\|"); if (p.length<1) continue;
        if ("POS".equals(p[0]) && p.length>=7){
          String id=p[1]; double x=Double.parseDouble(p[3]), y=Double.parseDouble(p[4]), hdg=Double.parseDouble(p[5]);
          AllyPos ap=ally.getOrDefault(id,new AllyPos());
          ap.x=x; ap.y=y; ap.hdg=hdg; ap.last=tick; ap.id=id; ally.put(id,ap);
        } else if ("TXY".equals(p[0]) && p.length>=6){
          double x=Double.parseDouble(p[3]), y=Double.parseDouble(p[4]), rad=Double.parseDouble(p[5]);
          updateTracks(x,y,rad,false);
        }
      }catch(Throwable ignore){}
    }
    ally.entrySet().removeIf(en -> tick - en.getValue().last > TTL_MSG);
  }

  private void updateTracks(double ex,double ey,double rad, boolean local){
    // 关联最近轨迹（180mm 门限）
    Track near=null; double best=1e18;
    for (Track t: tracks){ double dd=Math.hypot(t.x-ex,t.y-ey); if (dd<best){best=dd; near=t;} }
    if (near==null || best>180.0){ near=new Track(); near.init(ex,ey,rad,tick); tracks.add(near); }
    near.rad=rad; near.update(ex,ey,tick,local);
    if (local) lockLastObs=tick; // 供锁有效性使用
  }

  private void tryBroadcastTXY(double dirAbs,double dist,double rad){
    int k=(int)Math.floor(normalize(dirAbs)/TWO_PI*SECT); if (k<0)k=0; if (k>=SECT)k=SECT-1;
    if (tick - lastTxyBroadcast[k] < TXY_LIMIT) return;
    lastTxyBroadcast[k]=tick;
    double ex=posX + dist*Math.cos(dirAbs), ey=posY + dist*Math.sin(dirAbs);
    broadcast(String.format(java.util.Locale.ROOT,"TXY|%s|%d|%.1f|%.1f|%.1f",myId,tick,ex,ey,rad));
  }

  // ===== 里程计 / 传感 / 工具 =====
  private void applyOdo(){ if (Math.abs(odoPend)>1e-9){ posX += odoPend*Math.cos(odoHdg); posY += odoPend*Math.sin(odoHdg); odoPend=0.0; } }
  private void odoMove(){ odoPend += STEP; odoHdg=getHeading(); move(); }

  private IFrontSensorResult.Types frontType(){ try{ IFrontSensorResult r=detectFront(); return (r==null)?null:r.getObjectType(); }catch(Throwable t){return null;} }
  private boolean isObstacle(IFrontSensorResult.Types t){
    if (t==null) return false;
    return t==IFrontSensorResult.Types.WALL || t==IFrontSensorResult.Types.Wreck
        || t==IFrontSensorResult.Types.TeamMainBot || t==IFrontSensorResult.Types.TeamSecondaryBot;
  }

  private static double normalize(double a){ a%=TWO_PI; if (a<0) a+=TWO_PI; return a; }
  private static double angDiff(double from,double to){ double d=normalize(to-from); if(d>Math.PI)d-=TWO_PI; if(d<=-Math.PI)d+=TWO_PI; return d; }
}
