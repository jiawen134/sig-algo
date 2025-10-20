package algorithms;

import java.util.ArrayList;
import java.lang.reflect.Method;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IRadarResult;
import characteristics.IFrontSensorResult;

public class SwiftScout extends Brain {

  private enum Mode { SWEEP, ENGAGE, EVADE, AVOID }
  private Mode mode = Mode.SWEEP;

  private static final double TWO_PI = Math.PI * 2;

  private static final double STEP_TURN =
      Math.max(Parameters.teamASecondaryBotStepTurnAngle, Parameters.teamBSecondaryBotStepTurnAngle);

  private static final double AIM_TOL      = STEP_TURN*1.1;
  private static final int    LOCAL_RELOAD = Math.max(8, Parameters.bulletFiringLatency-4);

  private static final int LOST_TIMEOUT = 40;
  private static final int ORBIT_PERIOD = 20;
  private static final int EVADE_TICKS  = 16;
  private static final int AVOID_STICK  = 12;

  private boolean orbitRight=true, preferRight=true, avoidRight=true;
  private int tick=0, reload=0, lostTicks=LOST_TIMEOUT+1;
  private int orbitPhase=0, avoidTicks=0, evadeTicks=0;

  private double targetDir=Double.NaN, lastSeenDir=Double.NaN;
  private double lastEnemyDir=Double.NaN, enemyAngVel=0.0;
  private double lastHealth=-1, threatDir=Double.NaN, lastRawDir=Double.NaN;

  @Override
  public void activate() {
    mode=Mode.SWEEP;
    tick=0; reload=0; orbitPhase=0;
    avoidTicks=0; evadeTicks=0;
    preferRight=true; orbitRight=true; avoidRight=true;
    targetDir=Double.NaN; lastSeenDir=getHeading();
    lastEnemyDir=Double.NaN; enemyAngVel=0.0;
    lastHealth=getHealth(); threatDir=Double.NaN; lastRawDir=Double.NaN;
    sendLogMessage("SwiftScout (fix): activated.");
  }

  @Override
  public void step() {
    tick++; if (reload>0) reload--;

    // 0) 被击中 -> EVADE
    double hp = getHealth();
    if (lastHealth>=0 && hp<lastHealth) {
      double hit = tryGetLastHitDirection();
      if (Double.isNaN(hit)) hit = !Double.isNaN(lastSeenDir) ? lastSeenDir : normalizeAngle(getHeading()+Math.PI);
      threatDir = hit; evadeTicks=EVADE_TICKS; mode=Mode.EVADE; orbitRight=!orbitRight;
    }
    lastHealth=hp;

    // 1) 避障（仅拦墙/友军）
    if (frontBlockedWallOrTeam()){
      mode=Mode.AVOID;
      if (avoidTicks==0){ avoidRight=preferRight; preferRight=!preferRight; }
      avoidTicks=AVOID_STICK;
    }else if (mode==Mode.AVOID){
      if (avoidTicks>0) avoidTicks--;
      if (avoidTicks==0) mode=Mode.SWEEP;
    }

    // 1.5) 近距反应射击（敌在正前方且无墙/友军挡）
    if (mode!=Mode.AVOID && opponentAhead() && canFireNow()){
      double j = (Math.random()-0.5)*AIM_TOL*1.0;
      fire(normalizeAngle(getHeading() + j));
      reload = LOCAL_RELOAD;
    }

    // 2) 选敌
    if (mode!=Mode.AVOID){
      ArrayList<IRadarResult> radar = detectRadar();
      IRadarResult lock = pickTarget(radar);
      if (lock!=null){
        double raw = lock.getObjectDirection();
        double cur = toAbsoluteAngle(raw);

        if (!Double.isNaN(lastEnemyDir)){
          double d = signedAngleDiff(lastEnemyDir, cur);
          enemyAngVel = 0.6*enemyAngVel + 0.4*d;
        }
        lastEnemyDir=cur;
        targetDir=cur; lastSeenDir=cur; lostTicks=0;
        if (mode!=Mode.EVADE) mode=Mode.ENGAGE;
      }else{
        lostTicks++;
        if (lostTicks>LOST_TIMEOUT && mode!=Mode.EVADE){
          targetDir=Double.NaN; lastEnemyDir=Double.NaN; enemyAngVel=0.0; mode=Mode.SWEEP;
        }else if (mode!=Mode.EVADE){
          targetDir=lastSeenDir; mode=Mode.ENGAGE;
        }
      }
    }

    // 3) 行为
    switch(mode){
      case AVOID:  avoidStep();  break;
      case EVADE:  evadeStep();  break;
      case ENGAGE: engageStep(); break;
      case SWEEP:
      default:     sweepStep();  break;
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
    if (canFireNow()){
      double j = (Math.random()-0.5)*AIM_TOL*1.3;
      fire(normalizeAngle(threatDir + j));
      reload = LOCAL_RELOAD;
    }
    move();
    if (--evadeTicks<=0) mode = Double.isNaN(targetDir)?Mode.SWEEP:Mode.ENGAGE;
  }

  private void engageStep(){
    double lead = normalizeAngle(targetDir + enemyAngVel*6.0);

    // 机头朝切线方向绕圈
    double tangent = normalizeAngle(lead + (orbitRight? +0.5 : -0.5)*Math.PI);
    double diff = signedAngleDiff(getHeading(), tangent);
    if (Math.abs(diff) > AIM_TOL){
      stepTurn(diff>0?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
      return;
    }

    // ★ 不再要求机头对准才能开火：直接按绝对角开火
    if (canFireNow()){
      double j = (Math.random()-0.5)*AIM_TOL*1.0;
      fire(normalizeAngle(lead + j));
      reload = LOCAL_RELOAD;
    }

    orbitPhase = (orbitPhase+1)%ORBIT_PERIOD;
    move();
    if (orbitPhase == ORBIT_PERIOD/2) {
      stepTurn(orbitRight?Parameters.Direction.LEFT:Parameters.Direction.RIGHT);
    }
  }

  private void sweepStep(){
    if (tick%25==0) preferRight=!preferRight;
    stepTurn(preferRight?Parameters.Direction.RIGHT:Parameters.Direction.LEFT);
    move();
  }

  // ======= 传感/工具 =======
  private boolean canFireNow(){
    if (reload>0) return false;
    return !frontBlockedWallOrTeam(); // 友军/墙挡住就别开火
  }

  private IFrontSensorResult.Types frontType(){
    try{
      Object fr = detectFront(); if (fr==null) return null;
      return (fr instanceof IFrontSensorResult)
          ? ((IFrontSensorResult)fr).getObjectType()
          : (IFrontSensorResult.Types) fr;
    }catch(Throwable e){ return null; }
  }

  // ★ 仅将 WALL/Team* 视为阻塞；Opponent* 不阻塞（允许打）
  private boolean frontBlockedWallOrTeam(){
    IFrontSensorResult.Types t = frontType();
    if (t==null) return false;
    String n = t.name();
    if (n.equalsIgnoreCase("Nothing")) return false;
    if (n.equalsIgnoreCase("WALL")) return true;
    if (n.startsWith("Team")) return true;
    return false;
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
      return candAbs;
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
