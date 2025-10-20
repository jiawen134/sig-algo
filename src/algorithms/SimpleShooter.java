/* ******************************************************
 * Simovies - Simple Shooter for Testing
 * ******************************************************/
package algorithms;

import java.util.ArrayList;
import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

public class SimpleShooter extends Brain {
    private int tick = 0;
    
    public SimpleShooter() { 
        super(); 
    }
    
    @Override
    public void activate() {
        tick = 0;
        sendLogMessage("SimpleShooter: ACTIVATED");
    }
    
    @Override
    public void step() {
        tick++;
        
        // 1. 扫描敌人
        ArrayList<IRadarResult> radar = detectRadar();
        
        if (radar != null && radar.size() > 0) {
            sendLogMessage("SimpleShooter: Radar detected " + radar.size() + " objects");
        }
        
        for (IRadarResult r : radar) {
            sendLogMessage("SimpleShooter: Object type = " + r.getObjectType());
            
            if (r.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                r.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                
                // 发现敌人！
                double relativeDir = r.getObjectDirection();
                double absoluteDir = getHeading() + relativeDir; // Convert to absolute world angle
                double dist = r.getObjectDistance();
                String type = r.getObjectType().toString();
                
                sendLogMessage("SimpleShooter: ENEMY FOUND! Type=" + type + 
                             ", RelDir=" + Math.toDegrees(relativeDir) + 
                             " deg, AbsDir=" + Math.toDegrees(absoluteDir) + 
                             " deg, Dist=" + dist + "mm");
                
                // Fire at absolute world angle!
                fire(absoluteDir);
                sendLogMessage("SimpleShooter: >>>FIRED at " + Math.toDegrees(absoluteDir) + " deg<<<");
                
                // 简单移动
                move();
                return;
            }
        }
        
        // 2. 没有敌人 - 巡逻
        if (tick % 20 == 0) {
            sendLogMessage("SimpleShooter: Patrolling... tick=" + tick);
        }
        
        IFrontSensorResult front = detectFront();
        if (front.getObjectType() != IFrontSensorResult.Types.NOTHING) {
            stepTurn(Parameters.Direction.RIGHT);
        } else {
            move();
        }
    }
}

