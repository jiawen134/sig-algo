package algorithms;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

import java.util.ArrayList;

public class AegisMain extends Brain {
private static final double ANGLEPRECISION = 0.01;
private static final double FIREANGLEPRECISION = Math.PI / 6;


private static final int ALPHA = 0x1EADDA;
private static final int BETA = 0x5EC0;
private static final int GAMMA = 0x333;
private static final int TEAM = 0xBADDAD;

private static final int FIRE = 0xB52;
private static final int OVER = 0xC00010FF;

private static final int MOVETASK = 1;
private static final int AVOIDTASK = 2;

private int state;
private double oldAngle;
private double myX, myY;
private boolean isMoving;
private int whoAmI;
private int fireRythm, countDown;
private double targetX, targetY;
private boolean fireOrder;
private boolean friendlyFire;

// Variables pour l'évitement
private boolean avoiding = false;
private double avoidanceDir;
private int avoidanceSteps = 0;

public AegisMain() { super(); }

public void activate() {
    whoAmI = GAMMA;
    for (IRadarResult o : detectRadar())
        if (isSameDirection(o.getObjectDirection(), Parameters.NORTH)) whoAmI = ALPHA;
    for (IRadarResult o : detectRadar())
        if (isSameDirection(o.getObjectDirection(), Parameters.SOUTH) && whoAmI != GAMMA) whoAmI = BETA;

    if (whoAmI == GAMMA) {
        myX = Parameters.teamAMainBot1InitX;
        myY = Parameters.teamAMainBot1InitY;
    } else if (whoAmI == BETA) {
        myX = Parameters.teamAMainBot2InitX;
        myY = Parameters.teamAMainBot2InitY;
    } else {
        myX = Parameters.teamAMainBot3InitX;
        myY = Parameters.teamAMainBot3InitY;
    }

    state = MOVETASK;
    isMoving = false;
    fireOrder = false;
    fireRythm = 0;
    targetX = 1500;
    targetY = 1000;
    sendLogMessage("Main bot activated (" + whoAmI + ")");
}

public void step() {
    // --- ODOMÉTRIE ---
    if (isMoving) {
        myX += Parameters.teamAMainBotSpeed * Math.cos(getHeading());
        myY += Parameters.teamAMainBotSpeed * Math.sin(getHeading());
        isMoving = false;
    }

    // --- RÉCEPTION MESSAGES ---
    ArrayList<String> messages = fetchAllMessages();
    for (String m : messages)
        if (Integer.parseInt(m.split(":")[1]) == TEAM) process(m);

    // --- DÉTECTION RADAR ---
    friendlyFire = true;
    for (IRadarResult o : detectRadar()) {
        if (o.getObjectType() == IRadarResult.Types.OpponentMainBot ||
            o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
            double enemyX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
            double enemyY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
            broadcast(whoAmI + ":" + TEAM + ":" + FIRE + ":" + enemyX + ":" + enemyY + ":" + OVER);
            sendLogMessage("Enemy detected! (" + (int)enemyX + "," + (int)enemyY + ")");
        }
        if (fireOrder && onTheWay(o.getObjectDirection())) {
            friendlyFire = false;
        }
    }

    // --- PHASE D'ÉVITEMENT ---
    if (avoiding) {
        if (!isSameDirection(getHeading(), avoidanceDir)) {
            stepTurnShortest(avoidanceDir);
            return;
        } else if (avoidanceSteps > 0) {
            move();
            updatePosition(true);
            avoidanceSteps--;
            if (avoidanceSteps <= 0) {
                avoiding = false;
                state = MOVETASK;
                sendLogMessage("Obstacle avoided, resuming path.");
            }
            return;
        }
    }

    // --- DÉTECTION OBSTACLE ---
    if (checkForObstacles()) return;

    // --- TIR ---
    if (fireOrder && fireRythm == 0 && friendlyFire) {
        firePosition(targetX, targetY);
        sendLogMessage("Firing at (" + (int)targetX + "," + (int)targetY + ")");
        fireRythm++;
        return;
    }
    fireRythm++;
    if (fireRythm >= 3) fireRythm = 0;

    // --- MOUVEMENT NORMAL ---
    if (state == MOVETASK) {
        move();
        updatePosition(true);
    }
}

private boolean checkForObstacles() {
    IFrontSensorResult front = detectFront();
    if (front.getObjectType() == IFrontSensorResult.Types.WALL ||
        front.getObjectType() == IFrontSensorResult.Types.Wreck) {
        avoidanceDir = normalizeAngle(getHeading() + (Math.random() > 0.5 ? Math.PI / 2 : -Math.PI / 2));
        avoidanceSteps = 25;
        avoiding = true;
        state = AVOIDTASK;
        sendLogMessage("Avoiding obstacle...");
        return true;
    }
    return false;
}

private void process(String message) {
    if (Integer.parseInt(message.split(":")[2]) == FIRE) {
        fireOrder = true;
        targetX = Double.parseDouble(message.split(":")[3]);
        targetY = Double.parseDouble(message.split(":")[4]);
        sendLogMessage("Received FIRE order → target (" + (int)targetX + "," + (int)targetY + ")");
    }
}

private void firePosition(double x, double y) {
    if (myX <= x) fire(Math.atan((y - myY) / (x - myX)));
    else fire(Math.PI + Math.atan((y - myY) / (x - myX)));
}

private boolean onTheWay(double angle) {
    if (myX <= targetX)
        return isRoughlySameDirection(angle, Math.atan((targetY - myY) / (targetX - myX)));
    else
        return isRoughlySameDirection(angle, Math.PI + Math.atan((targetY - myY) / (targetX - myX)));
}

private void stepTurnShortest(double dir) {
    double diff = normalizeAngle(dir - getHeading());
    if (Math.sin(diff) > 0) stepTurn(Parameters.Direction.RIGHT);
    else stepTurn(Parameters.Direction.LEFT);
}

private void updatePosition(boolean forward) {
    double step = Parameters.teamAMainBotSpeed;
    if (!forward) step = -step;
    myX += Math.cos(getHeading()) * step;
    myY += Math.sin(getHeading()) * step;
}

private boolean isSameDirection(double dir1, double dir2) {
    return Math.abs(normalizeAngle(dir1 - dir2)) < ANGLEPRECISION;
}

private boolean isRoughlySameDirection(double dir1, double dir2) {
    return Math.abs(normalizeAngle(dir1 - dir2)) < FIREANGLEPRECISION;
}

private double normalizeAngle(double a) {
    while (a <= -Math.PI) a += 2 * Math.PI;
    while (a > Math.PI) a -= 2 * Math.PI;
    return a;
}


}
