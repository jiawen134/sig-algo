/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Tactical Master Algorithm - Designed to Defeat Team B
 * Copyright (C) 2025 <Elite Tactician>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * ******************************************************/
package algorithms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

public class TacticalMaster extends Brain {
    //---CORE PARAMETERS---//
    private static final double HEADING_PRECISION = 0.001;
    private static final double OPTIMAL_ATTACK_DISTANCE = 250; // Optimal attack distance
    private static final double MIN_SAFE_DISTANCE = 100; // Minimum safe distance
    private static final double MAX_ATTACK_DISTANCE = 400; // Maximum attack distance
    private static final double CRITICAL_HEALTH = 80; // Higher threshold for better survival
    private static final double DODGE_DISTANCE = 150; // Distance to trigger dodge
    
    //---AI MODES---//
    private enum TacticalMode {
        SEEK_AND_DESTROY,  // Aggressive hunting
        SNIPER,            // Long-range precision
        FLANKING,          // Side attack
        EVASIVE,           // Dodging mode
        AMBUSH,            // Wait and ambush
        EMERGENCY_RETREAT  // Critical retreat
    }
    
    //---ENEMY TRACKING---//
    private static class Target {
        double direction;
        double distance;
        int lastSeen;
        boolean isMain;
        double predictedX;
        double predictedY;
        int hitCount;
        
        Target(double dir, double dist, boolean main) {
            this.direction = dir;
            this.distance = dist;
            this.lastSeen = 0;
            this.isMain = main;
            this.hitCount = 0;
        }
    }
    
    //---STATE VARIABLES---//
    private TacticalMode currentMode;
    private Map<String, Target> targets;
    private int modeTimer;
    private int fireCooldown;
    private int dodgeTimer;
    private int stuckTimer;
    private boolean isAggressive;
    private double lastX, lastY;
    private int stationaryCounter;
    private double combatZoneCenter;
    private boolean hasSeenEnemy;
    private int victoryCount;
    private double preferredAttackAngle;
    
    //---CONSTRUCTOR---//
    public TacticalMaster() { 
        super(); 
        targets = new HashMap<>();
        currentMode = TacticalMode.SEEK_AND_DESTROY;
        modeTimer = 0;
        fireCooldown = 0;
        dodgeTimer = 0;
        stuckTimer = 0;
        isAggressive = true;
        stationaryCounter = 0;
        combatZoneCenter = Math.PI; // Face enemy territory
        hasSeenEnemy = false;
        victoryCount = 0;
        preferredAttackAngle = 0;
    }
    
    //---MAIN LOGIC---//
    public void activate() {
        // Aggressive start - move towards enemy territory
        currentMode = TacticalMode.SEEK_AND_DESTROY;
        move();
        sendLogMessage("Tactical Master online. Hunt mode engaged.");
    }
    
    public void step() {
        // Update timers
        modeTimer++;
        fireCooldown++;
        dodgeTimer++;
        stuckTimer++;
        
        // Health management
        if (getHealth() <= CRITICAL_HEALTH) {
            if (currentMode != TacticalMode.EMERGENCY_RETREAT) {
                currentMode = TacticalMode.EMERGENCY_RETREAT;
                sendLogMessage("Critical damage! Emergency retreat!");
            }
        }
        
        // Detect and track enemies
        ArrayList<IRadarResult> radarResults = detectRadar();
        updateTargets(radarResults);
        
        // Anti-stuck mechanism
        checkStuck();
        
        // Execute tactical decision
        executeTacticalMode();
    }
    
    //---TARGET MANAGEMENT---//
    private void updateTargets(ArrayList<IRadarResult> radarResults) {
        // Age existing targets
        for (Target t : targets.values()) {
            t.lastSeen++;
        }
        
        // Remove stale targets
        targets.entrySet().removeIf(entry -> entry.getValue().lastSeen > 80);
        
        // Update with new radar data
        for (IRadarResult result : radarResults) {
            if (result.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                result.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                
                hasSeenEnemy = true;
                boolean isMain = (result.getObjectType() == IRadarResult.Types.OpponentMainBot);
                String key = isMain ? "MAIN" : "SECONDARY";
                
                Target target = new Target(result.getObjectDirection(), result.getObjectDistance(), isMain);
                targets.put(key, target);
            }
        }
    }
    
    private Target getPriorityTarget() {
        Target priority = null;
        double bestScore = -1;
        
        for (Target t : targets.values()) {
            if (t.lastSeen > 10) continue; // Skip stale targets
            
            // Scoring: prefer main bots, closer targets, and previously hit targets
            double score = 0;
            if (t.isMain) score += 100;
            score += Math.max(0, 500 - t.distance); // Closer is better
            score += t.hitCount * 50; // Prefer wounded targets
            
            if (score > bestScore) {
                bestScore = score;
                priority = t;
            }
        }
        
        return priority;
    }
    
    //---TACTICAL MODES---//
    private void executeTacticalMode() {
        switch (currentMode) {
            case SEEK_AND_DESTROY:
                executeSeekAndDestroy();
                break;
            case SNIPER:
                executeSniper();
                break;
            case FLANKING:
                executeFlanking();
                break;
            case EVASIVE:
                executeEvasive();
                break;
            case AMBUSH:
                executeAmbush();
                break;
            case EMERGENCY_RETREAT:
                executeEmergencyRetreat();
                break;
        }
    }
    
    //---SEEK AND DESTROY MODE---//
    private void executeSeekAndDestroy() {
        Target target = getPriorityTarget();
        
        if (target != null) {
            double distance = target.distance;
            double direction = target.direction;
            
            // Distance-based tactics
            if (distance < MIN_SAFE_DISTANCE) {
                // Too close - switch to evasive
                currentMode = TacticalMode.EVASIVE;
                dodgeTimer = 0;
                sendLogMessage("Enemy too close! Evasive maneuvers!");
                return;
            }
            
            if (distance > OPTIMAL_ATTACK_DISTANCE && distance < MAX_ATTACK_DISTANCE) {
                // Perfect range - sniper mode
                currentMode = TacticalMode.SNIPER;
                sendLogMessage("Optimal range. Sniper mode engaged!");
                return;
            }
            
            // Approach and attack
            if (canFire() && Math.random() < 0.9) {
                // Lead the target
                double leadAngle = calculateLeadAngle(target);
                fire(direction + leadAngle);
                fireCooldown = 0;
                target.hitCount++;
                sendLogMessage("Firing at target!");
            }
            
            // Movement towards target
            if (distance > OPTIMAL_ATTACK_DISTANCE) {
                // Too far - approach
                if (Math.abs(getHeading() - direction) > HEADING_PRECISION * 10) {
                    turnTowards(direction);
                } else {
                    // Check for obstacles before moving
                    if (!isPathBlocked()) {
                        move();
                    } else {
                        // Path blocked, try to maneuver
                        stepTurn(Math.random() < 0.5 ? Parameters.Direction.LEFT : Parameters.Direction.RIGHT);
                    }
                }
            } else {
                // Good distance - strafe
                strafeAroundTarget(direction);
            }
            
        } else {
            // No target - patrol aggressively
            aggressivePatrol();
            
            // Switch to ambush after 200 steps without target
            if (modeTimer > 200 && hasSeenEnemy) {
                currentMode = TacticalMode.AMBUSH;
                modeTimer = 0;
                sendLogMessage("Setting up ambush position.");
            }
        }
    }
    
    //---SNIPER MODE---//
    private void executeSniper() {
        Target target = getPriorityTarget();
        
        if (target == null || target.distance < OPTIMAL_ATTACK_DISTANCE - 50) {
            currentMode = TacticalMode.SEEK_AND_DESTROY;
            return;
        }
        
        // Precision positioning
        double direction = target.direction;
        
        // Fine-tune heading
        if (Math.abs(getHeading() - direction) > HEADING_PRECISION) {
            turnTowards(direction);
        }
        
        // Fire with high accuracy
        if (canFire() && Math.abs(getHeading() - direction) < HEADING_PRECISION * 5) {
            double leadAngle = calculateLeadAngle(target);
            fire(direction + leadAngle);
            fireCooldown = 0;
            target.hitCount++;
            sendLogMessage("Sniper shot fired!");
        }
        
        // Maintain distance
        if (target.distance < OPTIMAL_ATTACK_DISTANCE - 20) {
            moveBack();
        } else if (target.distance > OPTIMAL_ATTACK_DISTANCE + 50) {
            if (!isPathBlocked()) {
                move();
            } else {
                // Path blocked, sidestep
                stepTurn(Parameters.Direction.LEFT);
            }
        }
        
        // Return to seek after 100 steps
        if (modeTimer > 100) {
            currentMode = TacticalMode.SEEK_AND_DESTROY;
            modeTimer = 0;
        }
    }
    
    //---FLANKING MODE---//
    private void executeFlanking() {
        Target target = getPriorityTarget();
        
        if (target == null) {
            currentMode = TacticalMode.SEEK_AND_DESTROY;
            return;
        }
        
        // Move to side of target
        double flankAngle = target.direction + (preferredAttackAngle > 0 ? 0.3 : -0.3);
        
        if (Math.abs(getHeading() - flankAngle) > HEADING_PRECISION * 10) {
            turnTowards(flankAngle);
        } else {
            move();
        }
        
        // Fire when flanking
        if (canFire() && target.distance < MAX_ATTACK_DISTANCE) {
            fire(target.direction);
            fireCooldown = 0;
        }
        
        // Return to seek after flanking
        if (modeTimer > 80) {
            currentMode = TacticalMode.SEEK_AND_DESTROY;
            modeTimer = 0;
        }
    }
    
    //---EVASIVE MODE---//
    private void executeEvasive() {
        Target target = getPriorityTarget();
        
        if (target == null || target.distance > DODGE_DISTANCE * 2) {
            currentMode = TacticalMode.SEEK_AND_DESTROY;
            dodgeTimer = 0;
            return;
        }
        
        // Dodge pattern - zigzag backwards
        if (dodgeTimer % 10 < 5) {
            stepTurn(Parameters.Direction.LEFT);
        } else {
            stepTurn(Parameters.Direction.RIGHT);
        }
        
        moveBack();
        
        // Fire while evading
        if (canFire()) {
            fire(target.direction);
            fireCooldown = 0;
        }
        
        // Return to seek when safe
        if (dodgeTimer > 40 || target.distance > DODGE_DISTANCE * 2) {
            currentMode = TacticalMode.SEEK_AND_DESTROY;
            dodgeTimer = 0;
            sendLogMessage("Evasion complete. Re-engaging.");
        }
    }
    
    //---AMBUSH MODE---//
    private void executeAmbush() {
        // Find cover position
        IFrontSensorResult front = detectFront();
        
        if (front.getObjectType() == IFrontSensorResult.Types.WALL) {
            // Good ambush position
            Target target = getPriorityTarget();
            
            if (target != null && target.lastSeen < 5) {
                // Enemy detected - fire!
                if (canFire()) {
                    fire(target.direction);
                    fireCooldown = 0;
                    sendLogMessage("Ambush successful!");
                }
                
                // Switch to aggressive mode
                currentMode = TacticalMode.SEEK_AND_DESTROY;
                modeTimer = 0;
            }
        } else {
            // Move to find wall
            move();
        }
        
        // Timeout ambush
        if (modeTimer > 150) {
            currentMode = TacticalMode.SEEK_AND_DESTROY;
            modeTimer = 0;
        }
    }
    
    //---EMERGENCY RETREAT MODE---//
    private void executeEmergencyRetreat() {
        Target target = getPriorityTarget();
        
        // Retreat direction - away from enemies
        double retreatDirection = getHeading();
        if (target != null) {
            retreatDirection = target.direction + Math.PI; // Opposite direction
        }
        
        // Turn and run
        if (Math.abs(getHeading() - retreatDirection) > HEADING_PRECISION * 10) {
            turnTowards(retreatDirection);
        } else {
            moveBack();
        }
        
        // Fire back while retreating
        if (canFire() && target != null) {
            fire(target.direction);
            fireCooldown = 0;
        }
        
        // Return to combat when health recovers
        if (getHealth() > CRITICAL_HEALTH + 30) {
            currentMode = TacticalMode.SEEK_AND_DESTROY;
            sendLogMessage("Health stabilized. Re-engaging!");
        }
    }
    
    //---HELPER METHODS---//
    private void aggressivePatrol() {
        IFrontSensorResult front = detectFront();
        
        // Check for any obstacle including teammates
        if (front.getObjectType() == IFrontSensorResult.Types.WALL ||
            front.getObjectType() == IFrontSensorResult.Types.Wreck ||
            front.getObjectType() == IFrontSensorResult.Types.TeamMainBot ||
            front.getObjectType() == IFrontSensorResult.Types.TeamSecondaryBot) {
            // Smart turn - prefer turning towards enemy territory
            if (Math.random() < 0.7) {
                stepTurn(Parameters.Direction.LEFT);
            } else {
                stepTurn(Parameters.Direction.RIGHT);
            }
        } else {
            move();
        }
    }
    
    private void strafeAroundTarget(double targetDirection) {
        // Circle strafe pattern
        double strafeAngle = targetDirection + (modeTimer % 40 < 20 ? 0.2 : -0.2);
        
        if (Math.abs(getHeading() - strafeAngle) > HEADING_PRECISION * 10) {
            turnTowards(strafeAngle);
        } else {
            if (!isPathBlocked()) {
                move();
            } else {
                // Blocked while strafing, reverse strafe direction
                stepTurn(modeTimer % 40 < 20 ? Parameters.Direction.LEFT : Parameters.Direction.RIGHT);
            }
        }
    }
    
    private double calculateLeadAngle(Target target) {
        // Simple lead calculation based on distance
        double lead = 0;
        if (target.distance > 200) {
            lead = 0.05; // Lead more for distant targets
        } else if (target.distance > 100) {
            lead = 0.02;
        }
        return lead * (Math.random() < 0.5 ? 1 : -1);
    }
    
    private void turnTowards(double targetDirection) {
        double currentHeading = getHeading();
        double angleDiff = targetDirection - currentHeading;
        
        // Normalize angle
        while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
        while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
        
        if (angleDiff > 0) {
            stepTurn(Parameters.Direction.RIGHT);
        } else {
            stepTurn(Parameters.Direction.LEFT);
        }
    }
    
    private boolean canFire() {
        return fireCooldown >= Parameters.bulletFiringLatency;
    }
    
    private void checkStuck() {
        // Anti-stuck mechanism
        if (stuckTimer > 60) {
            sendLogMessage("Stuck detected! Forcing movement.");
            if (Math.random() < 0.5) {
                stepTurn(Parameters.Direction.RIGHT);
            } else {
                stepTurn(Parameters.Direction.LEFT);
            }
            move();
            stuckTimer = 0;
        }
    }
    
    private boolean isPathBlocked() {
        IFrontSensorResult front = detectFront();
        // Check for any obstacle including teammates
        return (front.getObjectType() == IFrontSensorResult.Types.WALL ||
                front.getObjectType() == IFrontSensorResult.Types.Wreck ||
                front.getObjectType() == IFrontSensorResult.Types.TeamMainBot ||
                front.getObjectType() == IFrontSensorResult.Types.TeamSecondaryBot ||
                front.getObjectType() == IFrontSensorResult.Types.OpponentMainBot ||
                front.getObjectType() == IFrontSensorResult.Types.OpponentSecondaryBot);
    }
}
