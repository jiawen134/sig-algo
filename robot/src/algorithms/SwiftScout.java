package algorithms;

import java.util.*;
import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

public class SwiftScout extends Brain {

  private static final double TWO_PI = Math.PI * 2.0;
  private static final int    POS_PERIOD = 25;
  private static final int    TXY_LIMIT  = 8;
  private static final int    TTL_MSG    = 80;
  private static final int    SECT       = 64;

  private static final double STEP         = Parameters.teamASecondaryBotSpeed; // 3
  private static final double BULLET_RANGE = Parameters.bulletRange;            // 1000
  private static final double DANGER_WIDTH = 90.0;                              // 主机射线廊道半宽
  private static final double BOUNDARY_MARGIN = 100.0;                          // 边界安全距离
  private static final double FIELD_WIDTH  = 3000.0;
  private static final double FIELD_HEIGHT = 2000.0;

  private int tick=0, lastPosBroadcast=-9999;
  private final String myId = "A_SCOUT_" + Integer.toHexString((int)(Math.random()*0xFFFF));
  private final int[] lastTxyBroadcast = new int[SECT];

  // 巡逻状态
  private int patrolPhase = 0;        // 巡逻阶段: 0=直行, 1=小转向
  private int moveStraightCount = 0;  // 直行计数器
  private int turnCount = 0;          // 转向计数器
  private double lastTurnDir = 1.0;   // 上次转向方向（1=右，-1=左）

  // 里程计
  private double posX, posY, odoPend=0.0, odoHdg=0.0;

  private static class AllyPos { double x,y,hdg; int last; String id; }
  private static class EnemyXY { double x,y,rad; int last; }
  private final Map<String, AllyPos> ally = new HashMap<>();
  private final List<EnemyXY> enemyXY = new ArrayList<>();

  private int evadeTicks=0; private double evadeAngle=0;

  @Override
  public void activate() {
    tick=0; Arrays.fill(lastTxyBroadcast, -9999);
    posX = Parameters.teamASecondaryBot1InitX;
    posY = Parameters.teamASecondaryBot1InitY;
    odoPend=0.0; odoHdg=getHeading();
    patrolPhase = 0;
    moveStraightCount = 0;
    turnCount = 0;
    lastTurnDir = 1.0;
    sendLogMessage("SwiftScout: Enhanced patrol + TXY broadcast + smart avoidance.");
  }

  @Override
  public void step() {
    tick++;
    applyOdo();

    // 周期广播自身坐标（供主机清线）
    if (tick - lastPosBroadcast >= POS_PERIOD){
      lastPosBroadcast=tick;
      String msg = String.format(java.util.Locale.ROOT, "POS|%s|%d|%.1f|%.1f|%.6f", myId, tick, posX, posY, getHeading());
      broadcast(msg);
    }

    processMessages();

    // 雷达→发现敌人就广播世界坐标
    ArrayList<IRadarResult> rs = detectRadar();
    if (rs!=null) for (IRadarResult r: rs){
      if (r.getObjectType()==IRadarResult.Types.OpponentMainBot ||
          r.getObjectType()==IRadarResult.Types.OpponentSecondaryBot){
        tryBroadcastTXY(r.getObjectDirection(), r.getObjectDistance(), r.getObjectRadius());
      }
    }

    // ===== 紧急避让：主机射线廊道 =====
    FireCorridor fc = inAnyMainFireCorridor(posX, posY);
    if (fc != null) {
      // 横向脱离并继续移动
      evadeAngle = Math.atan2(fc.uy, fc.ux) + (fc.side>0 ? +Math.PI/2 : -Math.PI/2);
      evadeTicks = 20;  // 增加脱离时间
    }
    if (evadeTicks > 0) {
      double diff = signedDiff(getHeading(), evadeAngle);
      if (Math.abs(diff) > 0.25) {
        stepTurn(diff>0?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
      } else {
        odoMove();  // 对准后立即移动
      }
      evadeTicks--; 
      return;
    }

    // ===== 边界检测与避让 =====
    if (isNearBoundary()) {
      handleBoundary();
      return;
    }

    // ===== 预测下一步：避免进入主机射线 =====
    double nx = posX + Math.cos(getHeading())*STEP*3;  // 预测3步后位置
    double ny = posY + Math.sin(getHeading())*STEP*3;
    if (willEnterAnyMainCorridor(nx, ny)) {
      // 小幅度转向，不要停止移动
      stepTurn(lastTurnDir>0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
      odoMove();  // 转向同时移动！
      return;
    }

    // ===== 前方障碍物检测 =====
    IFrontSensorResult.Types ft = frontType();
    if (frontIsBlock(ft)) {
      // 遇到障碍：转向并移动
      lastTurnDir = -lastTurnDir;  // 切换方向
      stepTurn(lastTurnDir>0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
      odoMove();
      return;
    }

    // ===== 智能巡逻模式 =====
    executePatrolStrategy();
  }

  // ===== 边界检测与处理 =====
  private boolean isNearBoundary() {
    return posX < BOUNDARY_MARGIN || posX > (FIELD_WIDTH - BOUNDARY_MARGIN) ||
           posY < BOUNDARY_MARGIN || posY > (FIELD_HEIGHT - BOUNDARY_MARGIN);
  }

  private void handleBoundary() {
    double hdg = getHeading();
    double targetAngle = hdg;
    
    // 根据位置计算远离边界的方向
    if (posX < BOUNDARY_MARGIN) {
      // 靠近左边界 → 向右（EAST）
      targetAngle = Parameters.EAST;
    } else if (posX > (FIELD_WIDTH - BOUNDARY_MARGIN)) {
      // 靠近右边界 → 向左（WEST）
      targetAngle = Parameters.WEST;
    } else if (posY < BOUNDARY_MARGIN) {
      // 靠近下边界 → 向上（NORTH）
      targetAngle = Parameters.NORTH;
    } else if (posY > (FIELD_HEIGHT - BOUNDARY_MARGIN)) {
      // 靠近上边界 → 向下（SOUTH）
      targetAngle = Parameters.SOUTH;
    }
    
    // 转向目标方向
    double diff = signedDiff(hdg, targetAngle);
    if (Math.abs(diff) > 0.15) {
      stepTurn(diff>0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
    }
    // 边界处理时也要移动，避免卡住
    odoMove();
  }

  // ===== 智能巡逻策略 =====
  private void executePatrolStrategy() {
    if (patrolPhase == 0) {
      // 直行阶段：移动15-25步
      odoMove();
      moveStraightCount++;
      
      if (moveStraightCount >= 20 + (tick % 10)) {  // 15-25步随机
        patrolPhase = 1;  // 切换到转向阶段
        moveStraightCount = 0;
        turnCount = 0;
      }
    } else {
      // 转向阶段：小幅转向2-4步
      stepTurn(lastTurnDir>0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
      odoMove();  // 转向同时移动
      turnCount++;
      
      if (turnCount >= 3) {
        patrolPhase = 0;  // 切换回直行阶段
        turnCount = 0;
        // 偶尔切换方向，制造螺旋效果
        if (tick % 100 == 0) lastTurnDir = -lastTurnDir;
      }
    }
  }

  // ===== 主机射线判定 =====
  private static class FireCorridor { double ux,uy; int side; }
  private FireCorridor inAnyMainFireCorridor(double x,double y){
    for (Map.Entry<String, AllyPos> en : ally.entrySet()){
      String id=en.getKey(); if (id==null || !id.startsWith("A_MAIN")) continue;
      AllyPos m=en.getValue(); if (tick - m.last > TTL_MSG) continue;
      for (EnemyXY e : enemyXY){
        if (tick - e.last > TTL_MSG) continue;
        double distME = Math.hypot(e.x - m.x, e.y - m.y);
        if (distME > BULLET_RANGE || distME < 1e-6) continue;
        double ux=(e.x-m.x)/distME, uy=(e.y-m.y)/distME;
        double vx=x-m.x, vy=y-m.y;
        double u =(vx*ux + vy*uy) / distME;
        if (u<=0 || u>=1) continue;
        double px=u*distME*ux, py=u*distME*uy;
        double perp=Math.hypot(vx-px, vy-py);
        if (perp < DANGER_WIDTH){
          FireCorridor fc=new FireCorridor();
          fc.ux=ux; fc.uy=uy;
          double sx=vx-px, sy=vy-py;
          fc.side = (ux*sy - uy*sx)>=0 ? +1 : -1;
          return fc;
        }
      }
    }
    return null;
  }
  private boolean willEnterAnyMainCorridor(double nx,double ny){ return inAnyMainFireCorridor(nx,ny)!=null; }

  // ===== 消息 / 广播 =====
  private void processMessages(){
    ArrayList<String> msgs = fetchAllMessages();
    if (msgs!=null) for (String m: msgs){
      try{
        String[] p = m.split("\\|"); if (p.length<1) continue;
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

  private void tryBroadcastTXY(double dirAbs,double dist,double rad){
    int k=sectorIndex(dirAbs);
    if (tick - lastTxyBroadcast[k] < TXY_LIMIT) return;
    lastTxyBroadcast[k]=tick;
    double ex=posX + dist*Math.cos(dirAbs);
    double ey=posY + dist*Math.sin(dirAbs);
    String msg = String.format(java.util.Locale.ROOT, "TXY|%s|%d|%.1f|%.1f|%.1f", myId, tick, ex, ey, rad);
    broadcast(msg);
  }

  // ===== 里程计 / 传感 / 工具 =====
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
}
