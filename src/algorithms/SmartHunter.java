/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Smart Hunter - Simple but Effective Anti-Berzerk Algorithm
 * Copyright (C) 2025.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * ******************************************************/
package algorithms;

import java.util.ArrayList;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

public class SmartHunter extends Brain {
    //---PARAMETERS---//
    private static final double HEADING_PRECISION = 0.001;
    private static final int SPREAD_OUT_TIMER = 50; // Time to spread out at start
    
    //---VARIABLES---//
    private int stepCounter;
    private double lastEnemyDirection;
    private int lastEnemyTime;
    private boolean spreadOutPhase;
    private double spreadDirection;
    private int turnCounter;
    private int fireCooldown;
    private int stuckCounter;
    private double lastHeading;
    
    //---CONSTRUCTOR---//
    public SmartHunter() { 
        super(); 
        stepCounter = 0;
        lastEnemyDirection = 0;
        lastEnemyTime = 999;
        spreadOutPhase = true;
        spreadDirection = Math.random() * Math.PI * 2; // Random initial direction
        turnCounter = 0;
        fireCooldown = 25; // Start ready to fire!
        stuckCounter = 0;
        lastHeading = 0;
    }
    
    //---MAIN LOGIC---//
    public void activate() {
        // Start with spread out movement
        lastHeading = getHeading();
        move();
        sendLogMessage("Smart Hunter online. Spreading out.");
    }
    
    public void step() {
        stepCounter++;
        fireCooldown++;
        lastEnemyTime++;
        
        // Check if stuck (heading not changing)
        if (Math.abs(getHeading() - lastHeading) < 0.001) {
            stuckCounter++;
        } else {
            stuckCounter = 0;
        }
        lastHeading = getHeading();
        
        // Anti-stuck mechanism
        if (stuckCounter > 30) {
            sendLogMessage("Stuck detected! Emergency maneuver.");
            // Random emergency turn
            if (Math.random() < 0.5) {
                stepTurn(Parameters.Direction.RIGHT);
                stepTurn(Parameters.Direction.RIGHT);
            } else {
                stepTurn(Parameters.Direction.LEFT);
                stepTurn(Parameters.Direction.LEFT);
            }
            stuckCounter = 0;
            return;
        }
        
        // Spread out phase - avoid clustering at start
        if (spreadOutPhase && stepCounter < SPREAD_OUT_TIMER) {
            executeSpreadOut();
            return;
        } else if (spreadOutPhase) {
            spreadOutPhase = false;
            sendLogMessage("Spread out complete. Engaging hunt mode.");
        }
        
        // Main combat logic
        executeCombat();
    }
    
    //---SPREAD OUT PHASE---//
    private void executeSpreadOut() {
        // IMPORTANT: Scan and fire FIRST, even during spread out
        ArrayList<IRadarResult> radarResults = detectRadar();
        IRadarResult enemy = findClosestEnemy(radarResults);
        
        if (enemy != null && canFire()) {
            // Enemy detected during spread out - fire immediately!
            fire(enemy.getObjectDirection());
            fireCooldown = 0;
            sendLogMessage("Spread out: Enemy spotted! Firing!");
        }
        
        IFrontSensorResult front = detectFront();
        
        // Check for any obstacle
        if (isBlocked(front)) {
            // Turn away from obstacle
            spreadDirection = getHeading() + (Math.random() - 0.5) * Math.PI;
            turnTowards(spreadDirection);
        } else {
            // Move in spread direction
            if (Math.abs(getHeading() - spreadDirection) > HEADING_PRECISION * 20) {
                turnTowards(spreadDirection);
            } else {
                move();
            }
        }
    }
    
    //---COMBAT LOGIC---//
    private void executeCombat() {
        // 1. Scan for enemies
        ArrayList<IRadarResult> radarResults = detectRadar();
        IRadarResult closestEnemy = findClosestEnemy(radarResults);
        
        if (closestEnemy != null) {
            lastEnemyDirection = closestEnemy.getObjectDirection();
            lastEnemyTime = 0;
            
            // AGGRESSIVE FIRING - always try to fire when enemy detected!
            if (canFire()) {
                fire(lastEnemyDirection);
                fireCooldown = 0;
                sendLogMessage("Combat: Firing at enemy!");
            } else {
                // Still cooling down, but ready soon
                sendLogMessage("Combat: Enemy locked! Reloading...");
            }
            
            // Movement strategy based on distance
            double distance = closestEnemy.getObjectDistance();
            
            if (distance < 150) {
                // Too close - back up while firing
                executeRetreat(lastEnemyDirection);
            } else if (distance > 350) {
                // Too far - approach
                executeApproach(lastEnemyDirection);
            } else {
                // Good range - strafe
                executeStrafe(lastEnemyDirection);
            }
        } else {
            // No enemy detected - aggressive patrol
            if (lastEnemyTime < 100) {
                // Recently saw enemy - move towards last known position
                executeApproach(lastEnemyDirection);
            } else {
                // No recent enemy - patrol
                executePatrol();
            }
        }
    }
    
    //---MOVEMENT MODES---//
    private void executeRetreat(double enemyDirection) {
        // Fire while retreating!
        if (canFire()) {
            fire(enemyDirection);
            fireCooldown = 0;
            sendLogMessage("Retreating and firing!");
        }
        
        // Back away from enemy
        double retreatDirection = enemyDirection + Math.PI; // Opposite direction
        
        if (Math.abs(getHeading() - retreatDirection) > HEADING_PRECISION * 20) {
            turnTowards(retreatDirection);
        } else {
            moveBack();
        }
    }
    
    private void executeApproach(double targetDirection) {
        IFrontSensorResult front = detectFront();
        
        if (isBlocked(front)) {
            // Path blocked - try to go around
            if (turnCounter % 10 < 5) {
                stepTurn(Parameters.Direction.LEFT);
            } else {
                stepTurn(Parameters.Direction.RIGHT);
            }
            turnCounter++;
        } else {
            // Clear path - move towards target
            if (Math.abs(getHeading() - targetDirection) > HEADING_PRECISION * 20) {
                turnTowards(targetDirection);
            } else {
                move();
                turnCounter = 0;
            }
        }
    }
    
    private void executeStrafe(double enemyDirection) {
        // Fire while strafing!
        if (canFire()) {
            fire(enemyDirection);
            fireCooldown = 0;
            sendLogMessage("Strafing and firing!");
        }
        
        // Circle strafe around enemy
        double strafeAngle = enemyDirection + (stepCounter % 60 < 30 ? 0.3 : -0.3);
        
        IFrontSensorResult front = detectFront();
        
        if (isBlocked(front)) {
            // Blocked - reverse strafe direction
            stepTurn(stepCounter % 60 < 30 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
        } else {
            if (Math.abs(getHeading() - strafeAngle) > HEADING_PRECISION * 20) {
                turnTowards(strafeAngle);
            } else {
                move();
            }
        }
    }
    
    private void executePatrol() {
        IFrontSensorResult front = detectFront();
        
        if (isBlocked(front)) {
            // Hit obstacle - turn
            if (Math.random() < 0.6) {
                stepTurn(Parameters.Direction.RIGHT);
            } else {
                stepTurn(Parameters.Direction.LEFT);
            }
        } else {
            // Clear path - keep moving
            if (stepCounter % 80 == 0) {
                // Occasionally change direction for better coverage
                if (Math.random() < 0.5) {
                    stepTurn(Parameters.Direction.RIGHT);
                }
            } else {
                move();
            }
        }
        
        // Opportunistic firing
        scanAndFire();
    }
    
    //---HELPER METHODS---//
    private IRadarResult findClosestEnemy(ArrayList<IRadarResult> radarResults) {
        IRadarResult closest = null;
        double minDistance = Double.MAX_VALUE;
        
        for (IRadarResult result : radarResults) {
            if (result.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                result.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                
                double distance = result.getObjectDistance();
                
                // Prioritize main bots
                if (result.getObjectType() == IRadarResult.Types.OpponentMainBot) {
                    distance *= 0.8; // Give main bots higher priority
                }
                
                if (distance < minDistance) {
                    minDistance = distance;
                    closest = result;
                }
            }
        }
        
        return closest;
    }
    
    private void scanAndFire() {
        ArrayList<IRadarResult> radarResults = detectRadar();
        
        for (IRadarResult result : radarResults) {
            if (result.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                result.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                
                if (canFire() && Math.random() < 0.7) {
                    fire(result.getObjectDirection());
                    fireCooldown = 0;
                }
                break;
            }
        }
    }
    
    private boolean isBlocked(IFrontSensorResult front) {
        // Check for any type of obstacle
        return (front.getObjectType() != IFrontSensorResult.Types.NOTHING);
    }
    
    private void turnTowards(double targetDirection) {
        double currentHeading = getHeading();
        double angleDiff = targetDirection - currentHeading;
        
        // Normalize angle to [-PI, PI]
        while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
        while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
        
        // Turn in the shorter direction
        if (angleDiff > 0) {
            stepTurn(Parameters.Direction.RIGHT);
        } else {
            stepTurn(Parameters.Direction.LEFT);
        }
    }
    
    private boolean canFire() {
        return fireCooldown >= Parameters.bulletFiringLatency;
    }
}

