package algorithms;

import java.util.ArrayList;
import java.lang.reflect.Method;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IRadarResult;
import characteristics.IFrontSensorResult;

public class AegisMain extends Brain {

  private enum Mode { PATROL, ENGAGE, EVADE, AVOID }
  private Mode mode = Mode.PATROL;

  private static final double TWO_PI = Math.PI * 2;

  private static final double STEP_TURN =
      Math.max(Parameters.teamAMainBotStepTurnAngle, Parameters.teamBMainBotStepTurnAngle);

  // --- 阈值（放宽开火门槛） ---
  private static final double TURN_ONLY_THR = 0.60;          // > ~34° 时仅转头
  private static final double AIM_TOL       = STEP_TURN * 1.2;
  private static final int    LOCAL_RELOAD  = Parameters.bulletFiringLatency;

  private static final int LOST_TIMEOUT = 60;
  private static final int KITE_PERIOD  = 28;
  private static final int EVADE_TICKS  = 22;
  private static final int AVOID_STICK  = 16;

  private boolean preferRight = true, avoidRight = true;
  private int tick=0, reload=0, lostTicks=LOST_TIMEOUT+1;
  private int kitePhase=0, avoidTicks=0, evadeTicks=0;

  private double targetDir=Double.NaN, lastSeenDir=Double.NaN;
  private double lastEnemyDir=Double.NaN, enemyAngVel=0.0;
  private double lastHealth=-1, threatDir=Double.NaN;
  private double lastRawDir=Double.NaN; // 雷达角自适应辅助

  @Override
  public void activate() {
    mode = Mode.PATROL;
    tick=0; reload=0; kitePhase=0;
    avoidTicks=0; evadeTicks=0;
    preferRight=true; avoidRight=true;
    targetDir=Double.NaN; lastSeenDir=getHeading();
    lastEnemyDir=Double.NaN; enemyAngVel=0.0;
    lastHealth=getHealth(); threatDir=Double.NaN; lastRawDir=Double.NaN;
    sendLogMessage("AegisMain (fix): activated.");
  }

  @Override
  public void step() {
    tick++; if (reload>0) reload--;

    // 0) 被击中 -> 侧向规避
    double hp = getHealth();
    if (lastHealth>=0 && hp<lastHealth) {
      double hit = tryGetLastHitDirection();
      if (Double.isNaN(hit)) hit = !Double.isNaN(lastSeenDir) ? lastSeenDir : normalizeAngle(getHeading()+Math.PI);
      threatDir = hit; evadeTicks = EVADE_TICKS; mode = Mode.EVADE;
    }
    lastHealth = hp;

    // 1) 避障（仅拦墙/友军，不拦敌人）
    if (frontBlockedWallOrTeam()) {
      mode = Mode.AVOID;
      if (avoidTicks==0){ avoidRight=preferRight; preferRight=!preferRight; }
      avoidTicks = AVOID_STICK;
    } else if (mode==Mode.AVOID) {
      if (avoidTicks>0) avoidTicks--;
      if (avoidTicks==0) mode = Mode.PATROL;
    }

    // 1.5) 近距反应射击：敌在正前方且无友军/墙挡，先朝机头点射
    if (mode != Mode.AVOID && opponentAhead() && canFireNow()) {
      double j = (Math.random()-0.5)*AIM_TOL*0.8;
      fire(normalizeAngle(getHeading() + j));
      reload = LOCAL_RELOAD;
    }

    // 2) 选敌/锁定（非 AVOID）
    if (mode!=Mode.AVOID) {
      ArrayList<IRadarResult> radar = detectRadar();
      IRadarResult lock = pickTarget(radar);
      if (lock!=null) {
        double raw = lock.getObjectDirection();
        double cur = toAbsoluteAngle(raw); // 自动判别并转绝对角

        if (!Double.isNaN(lastEnemyDir)) {
          double d = signedAngleDiff(lastEnemyDir, cur);
          enemyAngVel = 0.7*enemyAngVel + 0.3*d;
        }
        lastEnemyDir = cur;
        targetDir = cur; lastSeenDir = cur; lostTicks = 0;
        if (mode!=Mode.EVADE) mode = Mode.ENGAGE;
      } else {
        lostTicks++;
        if (lostTicks>LOST_TIMEOUT && mode!=Mode.EVADE) {
          targetDir=Double.NaN; lastEnemyDir=Double.NaN; enemyAngVel=0.0; mode=Mode.PATROL;
        } else if (mode!=Mode.EVADE) {
          targetDir = lastSeenDir; mode=Mode.ENGAGE;
        }
      }
    }

    // 3) 行为
    switch(mode){
      case AVOID:  avoidStep();  break;
      case EVADE:  evadeStep();  break;
      case ENGAGE: engageStep(); break;
      case PATROL:
      default:     patrolStep(); break;
    }
  }

  private void avoidStep(){
    stepTurn(avoidRight?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
    moveBack();
  }

  private void evadeStep(){
    double right = normalizeAngle(threatDir + 0.5*Math.PI);
    double left  = normalizeAngle(threatDir - 0.5*Math.PI);
    double goal  = (Math.abs(signedAngleDiff(getHeading(), right)) <
                    Math.abs(signedAngleDiff(getHeading(), left))) ? right : left;

    double diff = signedAngleDiff(getHeading(), goal);
    if (Math.abs(diff) > AIM_TOL){
      stepTurn(diff>0?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
      return;
    }
    // 侧移 + 压制
    if (canFireNow()){
      double j = (Math.random()-0.5)*AIM_TOL*1.1;
      fire(normalizeAngle(threatDir + j));
      reload = LOCAL_RELOAD;
    }
    move();
    if (--evadeTicks<=0) mode = Double.isNaN(targetDir)?Mode.PATROL:Mode.ENGAGE;
  }

  private void engageStep(){
    double lead = normalizeAngle(targetDir + enemyAngVel*6.0);
    double diff = signedAngleDiff(getHeading(), lead);
    if (Math.abs(diff) > TURN_ONLY_THR) {
      stepTurn(diff>0?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
      return; // 先对准
    }
    // 偏差中等也允许开火
    if (canFireNow()){
      double j = (Math.random()-0.5)*AIM_TOL*0.9;
      fire(normalizeAngle(lead + j));
      reload = LOCAL_RELOAD;
    }
    kitePhase = (kitePhase+1)%KITE_PERIOD;
    if (kitePhase < KITE_PERIOD/2) move(); else moveBack();
  }

  private void patrolStep(){
    if (tick%40==0) preferRight=!preferRight;
    stepTurn(preferRight?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
    move();
  }

  // ======= 传感/工具 =======
  private boolean canFireNow(){
    if (reload>0) return false;
    // 友军/墙在正前方则抑制射击
    return !frontBlockedWallOrTeam();
  }

  private IFrontSensorResult.Types frontType(){
    try{
      Object fr = detectFront(); if (fr==null) return null;
      return (fr instanceof IFrontSensorResult)
          ? ((IFrontSensorResult)fr).getObjectType()
          : (IFrontSensorResult.Types) fr;
    }catch(Throwable e){ return null; }
  }

  // ★ 仅拦“墙/友军”，不拦“敌人”
  private boolean frontBlockedWallOrTeam(){
    IFrontSensorResult.Types t = frontType();
    if (t==null) return false;
    String n = t.name();
    if (n.equalsIgnoreCase("Nothing")) return false;
    if (n.equalsIgnoreCase("WALL")) return true;
    if (n.startsWith("Team")) return true;
    return false; // Opponent* 不算阻塞
  }

  private boolean opponentAhead(){
    IFrontSensorResult.Types t = frontType();
    if (t==null) return false;
    String n = t.name();
    return n.startsWith("Opponent");
  }

  private IRadarResult pickTarget(ArrayList<IRadarResult> list){
    if (list==null || list.isEmpty()) return null;
    IRadarResult sec=null;
    for(IRadarResult r:list){
      if (r.getObjectType()==IRadarResult.Types.OpponentMainBot) return r;
      if (r.getObjectType()==IRadarResult.Types.OpponentSecondaryBot) sec=r;
    }
    return sec;
  }

  // 雷达角度自适应：在绝对(raw)与相对(heading+raw)中选更连贯者
  private double toAbsoluteAngle(double raw){
    double candAbs = normalizeAngle(raw);
    double candRel = normalizeAngle(getHeading() + raw);
    if (!Double.isNaN(lastSeenDir)) {
      double da = Math.abs(signedAngleDiff(lastSeenDir, candAbs));
      double dr = Math.abs(signedAngleDiff(lastSeenDir, candRel));
      lastRawDir = raw;
      return (da<=dr)?candAbs:candRel;
    } else {
      lastRawDir = raw;
      return candAbs; // 初值偏向绝对角
    }
  }

  private double tryGetLastHitDirection(){
    try{
      Method m = robotsimulator.Brain.class.getMethod("getLastHitDirection");
      Object v = m.invoke(this);
      if (v instanceof Double) return normalizeAngle((Double)v);
    }catch(Throwable ignore){}
    return Double.NaN;
  }

  private static double signedAngleDiff(double a, double b){
    double d = normalizeAngle(b-a);
    if (d> Math.PI)  d -= TWO_PI;
    if (d<=-Math.PI) d += TWO_PI;
    return d;
  }
  private static double normalizeAngle(double x){
    x%=TWO_PI; if (x<0) x+=TWO_PI; return x;
  }
}
