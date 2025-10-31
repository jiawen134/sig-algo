package algorithms;

import characteristics.IRadarResult;
import characteristics.IRadarResult.Types;
import characteristics.Parameters.Direction;
import java.util.ArrayList;
import java.util.HashMap;
import robotsimulator.Brain;

/**
 * Scout robot specialized in navigation and reconnaissance
 * Refactored with improved code structure while preserving original functionality
 */
public class Stage1Secondary extends Brain {
   
   // Team member identifiers
   private static final int SCOUT_UNIT_A = 2014683;
   private static final int SCOUT_UNIT_B = 24269;
   private static final int COMBAT_UNIT_A = 2010586;
   private static final int COMBAT_UNIT_B = 24256;
   private static final int COMBAT_UNIT_C = 819;
   private static final int COLLECTIVE_ID = 12246445;
   
   // Communication codes
   private static final int MSG_HOSTILE_CONTACT = 2898;
   private static final int MSG_STATUS_UPDATE = 32343;
   private static final int MSG_END_MARKER = -1073737473;
   private static final double CLASS_PRIMARY_HOSTILE = -1.431633921E9;
   private static final double CLASS_SECONDARY_HOSTILE = -21846.0;
   
   // Operational states
   private static final int STATE_ORIENT_NORTH = 1;
   private static final int STATE_ORIENT_SOUTH = 2;
   private static final int STATE_ORIENT_EAST = 3;
   private static final int STATE_ORIENT_WEST = 4;
   private static final int STATE_NAVIGATE = 5;
   private static final int STATE_INITIAL_ADVANCE = 6;
   private static final int STATE_EVADE = 7;
   private static final int STATE_ROTATE_LEFT = 8;
   private static final int STATE_FALLBACK = 9;
   private static final int STATE_ROTATE_RIGHT = 10;
   private static final int STATE_BEGIN_NORTH = 11;
   private static final int STATE_BEGIN_SOUTH = 22;
   private static final int STATE_TERMINATED = -1159983647;
   
   // Navigation precision
   private static final double BEARING_MATCH_TIGHT = 0.001;
   private static final double BEARING_MATCH_LOOSE = 0.01;
   
   // Position state
   private double myCoordX;
   private double myCoordY;
   private int myUnitId;
   private boolean westSideDeployment;
   
   // Movement state
   private boolean advancingFlag;
   private boolean retreatingFlag;
   private int operationalState;
   private int tickCount;
   private int evadeStartTick;
   private double desiredBearing;
   
   // Team coordination
   private HashMap<Integer, ArrayList<Double>> alliedUnits;
   private ArrayList<IRadarResult> hostileContacts;

   public Stage1Secondary() {
      this.alliedUnits = new HashMap<>();
      this.hostileContacts = new ArrayList<>();
      
      ArrayList<Double> defaultStatus = new ArrayList<>(2);
      defaultStatus.add(0.0);
      defaultStatus.add(0.0);
      
      this.alliedUnits.put(COMBAT_UNIT_A, new ArrayList<>(defaultStatus));
      this.alliedUnits.put(COMBAT_UNIT_B, new ArrayList<>(defaultStatus));
      this.alliedUnits.put(COMBAT_UNIT_C, new ArrayList<>(defaultStatus));
      this.alliedUnits.put(SCOUT_UNIT_A, new ArrayList<>(defaultStatus));
      this.alliedUnits.put(SCOUT_UNIT_B, new ArrayList<>(defaultStatus));
   }

   public void activate() {
      this.determineUnitRole();
      this.assessDeploymentSide();
      this.configureInitialCoordinates();
      this.initializeStateVariables();
   }

   private void determineUnitRole() {
      this.myUnitId = SCOUT_UNIT_A;
      
      for (IRadarResult scan : this.detectRadar()) {
         if (this.bearingsAreEqual(scan.getObjectDirection(), -1.5707963267948966)) {
            this.myUnitId = SCOUT_UNIT_B;
            break;
         }
      }
   }

   private void assessDeploymentSide() {
      this.westSideDeployment = true;
      
      for (IRadarResult scan : this.detectRadar()) {
         if (this.bearingsAreEqual(scan.getObjectDirection(), 0.0)) {
            this.westSideDeployment = false;
            break;
         }
      }
   }

   private void configureInitialCoordinates() {
      if (this.westSideDeployment) {
         if (this.myUnitId == SCOUT_UNIT_A) {
            this.myCoordX = 500.0;
            this.myCoordY = 800.0;
            this.operationalState = STATE_BEGIN_NORTH;
         } else {
            this.myCoordX = 500.0;
            this.myCoordY = 1200.0;
            this.operationalState = STATE_BEGIN_SOUTH;
         }
      } else {
         if (this.myUnitId == SCOUT_UNIT_A) {
            this.myCoordX = 2500.0;
            this.myCoordY = 800.0;
            this.operationalState = STATE_BEGIN_NORTH;
         } else {
            this.myCoordX = 2500.0;
            this.myCoordY = 1200.0;
            this.operationalState = STATE_BEGIN_SOUTH;
         }
      }
      
      this.westSideDeployment = (this.myCoordX != 2500.0);
   }

   private void initializeStateVariables() {
      this.advancingFlag = false;
      this.retreatingFlag = false;
      this.tickCount = 0;
      this.evadeStartTick = 0;
   }

   public void step() {
      this.tickCount++;
      
      if (this.getHealth() == 0.0) {
         this.operationalState = STATE_TERMINATED;
      }

      this.updateCoordinateTracking();
      this.transmitTelemetry();
      this.processRadarInformation();
   }

   private void updateCoordinateTracking() {
      if (this.advancingFlag) {
         this.myCoordX += 3.0 * Math.cos(this.getHeading());
         this.myCoordY += 3.0 * Math.sin(this.getHeading());
         this.applyArenaBounds();
         this.advancingFlag = false;
      }

      if (this.retreatingFlag) {
         double currentBearing = this.getBearingStandardized();
         this.myCoordX -= Math.cos(currentBearing);
         this.myCoordY -= Math.sin(currentBearing);
         this.applyArenaBounds();
         this.retreatingFlag = false;
      }
   }

   private void applyArenaBounds() {
      if (this.myCoordX < 0.0) this.myCoordX = 0.0;
      if (this.myCoordX > 3000.0) this.myCoordX = 3000.0;
      if (this.myCoordY < 0.0) this.myCoordY = 0.0;
      if (this.myCoordY > 2000.0) this.myCoordY = 2000.0;
   }

   private void transmitTelemetry() {
      String unitDesignation = (this.myUnitId == SCOUT_UNIT_A) ? "Scout-1" : "Scout-2";
      this.sendLogMessage("[" + unitDesignation + "] Coords: (" + 
                         (int)this.myCoordX + "," + (int)this.myCoordY + ") | Status: " + this.operationalState);
   }

   private void processRadarInformation() {
      boolean hostileDetected = false;
      
      for (IRadarResult contact : this.detectRadar()) {
         if (contact.getObjectType() == Types.OpponentMainBot || 
             contact.getObjectType() == Types.OpponentSecondaryBot) {
            hostileDetected = true;
            this.relayHostilePosition(contact);
            this.hostileContacts.add(contact);
         }
      }
      
      if (!hostileDetected) {
         this.executePrimaryLogic();
      } else {
         this.executePrimaryLogic();
      }
   }

   private void relayHostilePosition(IRadarResult contact) {
      double hostileX = this.myCoordX + contact.getObjectDistance() * Math.cos(contact.getObjectDirection());
      double hostileY = this.myCoordY + contact.getObjectDistance() * Math.sin(contact.getObjectDirection());
      
      double hostileClass = (contact.getObjectType() == Types.OpponentMainBot) ? CLASS_PRIMARY_HOSTILE : CLASS_SECONDARY_HOSTILE;
      String alert = this.myUnitId + ":" + COLLECTIVE_ID + ":" + MSG_HOSTILE_CONTACT + ":" + 
                      hostileClass + ":" + hostileX + ":" + hostileY + ":" + MSG_END_MARKER;
      this.broadcast(alert);
   }

   private void executePrimaryLogic() {
      this.relayCurrentStatus();
      this.analyzeThreats();
      this.enforceArenaBoundaries();
      this.runStateMachine();
      
      if (this.operationalState == STATE_TERMINATED) {
         return;
      }
   }

   private void relayCurrentStatus() {
      String statusReport = this.myUnitId + ":" + COLLECTIVE_ID + ":" + MSG_STATUS_UPDATE + ":" + 
                      this.myCoordX + ":" + this.myCoordY + ":" + this.getBearingStandardized() + ":" + MSG_END_MARKER;
      this.broadcast(statusReport);
   }

   private void analyzeThreats() {
      this.hostileContacts.clear();
      
      for (IRadarResult contact : this.detectRadar()) {
         boolean primaryThreat = contact.getObjectType() == Types.OpponentMainBot && 
                                   contact.getObjectDistance() <= 400.0;
         boolean secondaryThreat = contact.getObjectType() == Types.OpponentSecondaryBot && 
                                    contact.getObjectDistance() <= 350.0;
         
         if (primaryThreat || secondaryThreat) {
            this.hostileContacts.add(contact);
            if (this.operationalState == STATE_NAVIGATE) {
               this.operationalState = STATE_EVADE;
            }
         }
         
         boolean immediateDanger = contact.getObjectDistance() < 120.0 && 
                          contact.getObjectType() != Types.BULLET && 
                          this.operationalState == STATE_NAVIGATE;
         
         if (immediateDanger) {
            this.operationalState = STATE_FALLBACK;
            this.evadeStartTick = this.tickCount;
         }
      }
   }

   private void enforceArenaBoundaries() {
      if (this.myCoordX <= 50.0) {
         this.operationalState = this.checkBearing(0.0) ? STATE_NAVIGATE : STATE_ORIENT_EAST;
         return;
      }

      if (this.myCoordX >= 2950.0) {
         this.operationalState = this.checkBearing(Math.PI) ? STATE_NAVIGATE : STATE_ORIENT_WEST;
         return;
      }

      if (this.myCoordY <= 50.0) {
         this.operationalState = this.checkBearing(1.5707963267948966) ? STATE_NAVIGATE : STATE_ORIENT_SOUTH;
         return;
      }

      if (this.myCoordY >= 1950.0) {
         if (this.checkBearing(-1.5707963267948966)) {
            this.operationalState = STATE_NAVIGATE;
         } else {
            this.operationalState = STATE_ORIENT_NORTH;
         }
      }
   }

   private void runStateMachine() {
      switch (this.operationalState) {
         case STATE_INITIAL_ADVANCE:
            this.processInitialAdvance();
            break;
         case STATE_BEGIN_NORTH:
         case STATE_BEGIN_SOUTH:
            this.processInitialOrientation();
            break;
         case STATE_ORIENT_NORTH:
         case STATE_ORIENT_SOUTH:
         case STATE_ORIENT_EAST:
         case STATE_ORIENT_WEST:
            this.processBoundaryAlignment();
            break;
         case STATE_NAVIGATE:
            this.processNavigationMode();
            break;
         case STATE_ROTATE_LEFT:
         case STATE_ROTATE_RIGHT:
            this.processRotationMode();
            break;
         case STATE_FALLBACK:
            this.processFallbackMode();
            break;
         case STATE_EVADE:
            this.processEvasionMode();
            break;
      }
   }

   private void processInitialAdvance() {
      this.advanceUnit();
      
      if (this.myUnitId == SCOUT_UNIT_B) {
         if (this.myCoordY > 1800.0) {
            this.operationalState = this.westSideDeployment ? STATE_ORIENT_EAST : STATE_ORIENT_WEST;
         }
      } else {
         if (this.myCoordY < 500.0) {
            this.operationalState = this.westSideDeployment ? STATE_ORIENT_EAST : STATE_ORIENT_WEST;
         }
      }
   }

   private void processInitialOrientation() {
      if (this.operationalState == STATE_BEGIN_NORTH) {
         if (!this.checkBearing(-1.5707963267948966)) {
            this.adjustBearingTo(-1.5707963267948966);
         } else {
            this.operationalState = STATE_INITIAL_ADVANCE;
            this.advanceUnit();
         }
      } else if (this.operationalState == STATE_BEGIN_SOUTH) {
         if (!this.checkBearing(1.5707963267948966)) {
            this.adjustBearingTo(1.5707963267948966);
         } else {
            this.operationalState = STATE_INITIAL_ADVANCE;
            this.advanceUnit();
         }
      }
   }

   private void processBoundaryAlignment() {
      double targetBearing = 0.0;
      boolean requiresRotation = false;
      
      if (this.operationalState == STATE_ORIENT_NORTH) {
         targetBearing = -1.5707963267948966;
         requiresRotation = !this.checkBearing(targetBearing);
      } else if (this.operationalState == STATE_ORIENT_SOUTH) {
         targetBearing = 1.5707963267948966;
         requiresRotation = !this.checkBearing(targetBearing);
      } else if (this.operationalState == STATE_ORIENT_EAST) {
         targetBearing = 0.0;
         requiresRotation = !this.checkBearing(targetBearing);
      } else if (this.operationalState == STATE_ORIENT_WEST) {
         targetBearing = Math.PI;
         requiresRotation = !this.checkBearing(targetBearing);
      }
      
      if (requiresRotation) {
         this.adjustBearingTo(targetBearing);
      } else {
         this.operationalState = STATE_NAVIGATE;
         this.advanceUnit();
      }
   }

   private void adjustBearingTo(double target) {
      double currentBearing = this.getBearingStandardized();
      
      if (target == 0.0 || target == Math.PI) {
         if (currentBearing < Math.PI && currentBearing > 0.0) {
            this.stepTurn(target == 0.0 ? Direction.LEFT : Direction.RIGHT);
         } else {
            this.stepTurn(target == 0.0 ? Direction.RIGHT : Direction.LEFT);
         }
      } else {
         boolean lowerHemisphere = !(currentBearing < 1.5707963267948966) && !(currentBearing > 4.71238898038469);
         
         if (target == -1.5707963267948966) {
            this.stepTurn(lowerHemisphere ? Direction.RIGHT : Direction.LEFT);
         } else {
            this.stepTurn(lowerHemisphere ? Direction.LEFT : Direction.RIGHT);
         }
      }
   }

   private void processNavigationMode() {
      if (this.detectFront().getObjectType() == characteristics.IFrontSensorResult.Types.WALL) {
         if (this.myUnitId != SCOUT_UNIT_B) {
            this.operationalState = STATE_ROTATE_LEFT;
            this.desiredBearing = this.getHeading() + -1.5707963267948966;
            this.stepTurn(Direction.LEFT);
         } else {
            this.navigateAroundObstacle();
         }
      } else {
         this.advanceUnit();
      }
   }

   private void navigateAroundObstacle() {
      boolean cornerPosition = (this.myCoordX > 2800.0 && this.myCoordY > 1800.0) ||
                         (this.myCoordX > 2800.0 && this.myCoordY < 200.0) ||
                         (this.myCoordX < 200.0 && this.myCoordY < 200.0) ||
                         (this.myCoordX < 200.0 && this.myCoordY > 1800.0);
      
      if (cornerPosition) {
         this.operationalState = STATE_ROTATE_LEFT;
         this.desiredBearing = this.getHeading() + -1.5707963267948966;
         this.stepTurn(Direction.LEFT);
         return;
      }
      
      boolean centerHorizontally = !(this.myCoordX > 2800.0) && !(this.myCoordX < 200.0);
      boolean centerVertically = !(this.myCoordY > 1800.0) && !(this.myCoordY < 200.0);
      
      if (centerHorizontally && centerVertically) {
         this.advanceUnit();
         return;
      }
      
      if (centerHorizontally) {
         if (!this.checkBearing(-1.5707963267948966) && !this.checkBearing(1.5707963267948966)) {
            this.advanceUnit();
            return;
         }
      }
      
      if (!this.checkBearing(0.0) && !this.checkBearing(Math.PI)) {
         this.advanceUnit();
         return;
      }
      
      this.operationalState = STATE_ROTATE_LEFT;
      this.desiredBearing = this.getHeading() + -1.5707963267948966;
      this.stepTurn(Direction.LEFT);
   }

   private void processRotationMode() {
      if (this.checkBearing(this.desiredBearing)) {
         this.operationalState = STATE_NAVIGATE;
         this.advanceUnit();
      } else {
         Direction rotationDirection = (this.operationalState == STATE_ROTATE_LEFT) ? Direction.LEFT : Direction.RIGHT;
         this.stepTurn(rotationDirection);
      }
   }

   private void processFallbackMode() {
      if (this.tickCount < this.evadeStartTick + 25) {
         this.retreatUnit();
      } else {
         if (Math.random() < 0.5) {
            this.operationalState = STATE_ROTATE_LEFT;
            this.desiredBearing = this.getHeading() + -1.5707963267948966;
            this.stepTurn(Direction.LEFT);
         } else {
            this.operationalState = STATE_ROTATE_RIGHT;
            this.desiredBearing = this.getHeading() + 1.5707963267948966;
            this.stepTurn(Direction.RIGHT);
         }
      }
   }

   private void processEvasionMode() {
      boolean edgeProximity = (this.myCoordX > 2900.0 || this.myCoordX < 100.0) && 
                          (this.myCoordY > 1900.0 || this.myCoordX < 100.0);
      
      if (edgeProximity) {
         this.operationalState = STATE_ROTATE_RIGHT;
         this.desiredBearing = this.getHeading() + 1.5707963267948966;
         this.stepTurn(Direction.RIGHT);
         return;
      }
      
      boolean centerHorizontally = !(this.myCoordX > 2900.0) && !(this.myCoordX < 100.0);
      boolean centerVertically = !(this.myCoordY > 1900.0) && !(this.myCoordY < 100.0);
      
      if (centerHorizontally && centerVertically) {
         this.executeEvasiveManeuver();
         return;
      }
      
      if (centerHorizontally) {
         if (!this.checkBearing(-1.5707963267948966) && !this.checkBearing(1.5707963267948966)) {
            this.executeEvasiveManeuver();
            return;
         }
      }
      
      if (!this.checkBearing(0.0) && !this.checkBearing(Math.PI)) {
         this.executeEvasiveManeuver();
         return;
      }
      
      this.operationalState = STATE_ROTATE_RIGHT;
      this.desiredBearing = this.getHeading() + 1.5707963267948966;
      this.stepTurn(Direction.RIGHT);
   }

   private void executeEvasiveManeuver() {
      this.moveBack();
      this.myCoordX -= 3.0 * Math.cos(this.getHeading());
      this.myCoordY -= 3.0 * Math.sin(this.getHeading());
      this.applyArenaBounds();
      
      if (this.hostileContacts.isEmpty()) {
         this.operationalState = STATE_NAVIGATE;
      }
   }

   private void advanceUnit() {
      this.advancingFlag = true;
      this.move();
   }

   private void retreatUnit() {
      this.retreatingFlag = true;
      this.moveBack();
   }

   private double standardizeBearing(double bearing) {
      while (bearing < 0.0) {
         bearing += 6.283185307179586;
      }
      while (bearing >= 6.283185307179586) {
         bearing -= 6.283185307179586;
      }
      return bearing;
   }

   private double getBearingStandardized() {
      return this.standardizeBearing(this.getHeading());
   }

   private boolean bearingsAreEqual(double bearing1, double bearing2) {
      return Math.abs(bearing1 - bearing2) < BEARING_MATCH_TIGHT;
   }

   private boolean checkBearing(double targetBearing) {
      return Math.abs(Math.sin(this.getHeading() - targetBearing)) < BEARING_MATCH_LOOSE;
   }
}
