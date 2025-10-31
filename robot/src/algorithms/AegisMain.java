package algorithms;

import characteristics.IRadarResult;
import characteristics.IFrontSensorResult.Types;
import characteristics.Parameters.Direction;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import robotsimulator.Brain;

/**
 * Advanced combat robot with multi-agent coordination capabilities
 * Refactored for improved code organization while maintaining identical behavior
 */
public class AegisMain extends Brain {

   // Robot identification constants
   private static final int BOT_ID_SCOUT_A = 2014683;
   private static final int BOT_ID_SCOUT_B = 24269;
   private static final int BOT_ID_MAIN_A = 2010586;
   private static final int BOT_ID_MAIN_B = 24256;
   private static final int BOT_ID_MAIN_C = 819;
   private static final int BROADCAST_TEAM_ID = 12246445;
   
   // Communication protocol codes
   private static final int MSG_ENEMY_SPOTTED = 2898;
   private static final int MSG_POSITION_SYNC = 32343;
   private static final int MSG_TERMINATOR = -1073737473;
   private static final double ENEMY_TYPE_PRIMARY = -1.431633921E9;
   private static final double ENEMY_TYPE_SECONDARY = -21846.0;
   
   // Operational modes
   private static final int OP_MODE_MOVING = 1;
   private static final int OP_MODE_STATIC_FIRE = 2;
   private static final int OP_MODE_MOBILE_FIRE = 3;
   private static final int OP_MODE_FALLBACK = 5;
   private static final int OP_MODE_TURN_LEFT = 6;
   private static final int OP_MODE_TURN_RIGHT = 7;
   private static final int OP_MODE_STARTUP = 8;
   private static final int OP_MODE_HUNTING = 9;
   private static final int OP_MODE_ORIENT_NORTH = 10;
   private static final int OP_MODE_ORIENT_SOUTH = 11;
   private static final int OP_MODE_ORIENT_EAST = 12;
   private static final int OP_MODE_ORIENT_WEST = 13;
   private static final int OP_MODE_DESTROYED = -1159983647;
   
   // Navigation precision thresholds
   private static final double ANGLE_MATCH_TIGHT = 0.001;
   private static final double ANGLE_MATCH_LOOSE = 0.01;
   private static final double POSITION_MATCH_TOLERANCE = 10.0;
   private static final double TRAJECTORY_CLEARANCE = 125.0;
   
   // Robot state - Position
   private double myPositionX;
   private double myPositionY;
   private int myIdentifier;
   private boolean isWestTeam;
   
   // Robot state - Movement
   private boolean flagMovingForward;
   private boolean flagMovingBackward;
   private int currentOperationMode;
   private int simulationTick;
   private int fallbackInitiatedTick;
   private double targetOrientation;
   
   // Robot state - Combat
   private double lockedTargetX;
   private double lockedTargetY;
   private boolean engagementActive;
   private int shotCounter;
   private int previousShotTick;
   private boolean seekingFireAngle;
   private String huntingDirection;
   
   // Team coordination data
   private HashMap<Integer, ArrayList<Double>> teamMemberStates;
   private ArrayList<ArrayList<Double>> detectedHostiles;
   
   // Utility objects
   private Random randomGenerator;
   private int continuousCounter;

   public AegisMain() {
      this.teamMemberStates = new HashMap<>();
      this.detectedHostiles = new ArrayList<>();
      this.randomGenerator = new Random();
      
      ArrayList<Double> initialState = new ArrayList<>(3);
      initialState.add(0.0);
      initialState.add(0.0);
      initialState.add(0.0);
      
      this.teamMemberStates.put(BOT_ID_MAIN_A, new ArrayList<>(initialState));
      this.teamMemberStates.put(BOT_ID_MAIN_B, new ArrayList<>(initialState));
      this.teamMemberStates.put(BOT_ID_MAIN_C, new ArrayList<>(initialState));
      this.teamMemberStates.put(BOT_ID_SCOUT_A, new ArrayList<>(initialState));
      this.teamMemberStates.put(BOT_ID_SCOUT_B, new ArrayList<>(initialState));
   }

   public void activate() {
      this.performIdentification();
      this.determineSideAssignment();
      this.initializeStartingLocation();
      this.resetAllStateVariables();
   }

   private void performIdentification() {
      this.myIdentifier = BOT_ID_MAIN_C;
      
      for (IRadarResult radarContact : this.detectRadar()) {
         if (checkAngleEquality(radarContact.getObjectDirection(), -1.5707963267948966)) {
            this.myIdentifier = BOT_ID_MAIN_A;
            break;
         }
      }
      
      if (this.myIdentifier != BOT_ID_MAIN_A) {
         for (IRadarResult radarContact : this.detectRadar()) {
            if (checkAngleEquality(radarContact.getObjectDirection(), 1.5707963267948966) && this.myIdentifier != BOT_ID_MAIN_C) {
               this.myIdentifier = BOT_ID_MAIN_B;
               break;
            }
         }
      }
   }

   private void determineSideAssignment() {
      this.isWestTeam = true;
      
      for (IRadarResult radarContact : this.detectRadar()) {
         double bearing = radarContact.getObjectDirection();
         if (bearing > 1.7278759594743864 && bearing < 4.5553093477052) {
            this.isWestTeam = false;
            this.broadcast("663121:663121:663121:663121:663121:663121:663121");
            break;
         }
      }
   }

   private void initializeStartingLocation() {
      if (this.isWestTeam) {
         if (this.myIdentifier == BOT_ID_MAIN_C) {
            this.myPositionX = 200.0;
            this.myPositionY = 800.0;
         } else if (this.myIdentifier == BOT_ID_MAIN_A) {
            this.myPositionX = 200.0;
            this.myPositionY = 1200.0;
         } else {
            this.myPositionX = 200.0;
            this.myPositionY = 1000.0;
         }
      } else {
         if (this.myIdentifier == BOT_ID_MAIN_C) {
            this.myPositionX = 2800.0;
            this.myPositionY = 800.0;
         } else if (this.myIdentifier == BOT_ID_MAIN_A) {
            this.myPositionX = 2800.0;
            this.myPositionY = 1200.0;
         } else {
            this.myPositionX = 2800.0;
            this.myPositionY = 1000.0;
         }
      }
   }

   private void resetAllStateVariables() {
      this.currentOperationMode = OP_MODE_STARTUP;
      this.flagMovingForward = false;
      this.flagMovingBackward = false;
      this.engagementActive = false;
      this.lockedTargetX = 0.0;
      this.lockedTargetY = 0.0;
      this.previousShotTick = 0;
      this.simulationTick = 0;
      this.fallbackInitiatedTick = 0;
      this.seekingFireAngle = false;
      this.continuousCounter = 0;
      this.huntingDirection = "";
   }

  public void step() {
      ArrayList<String> receivedMessages = this.fetchAllMessages();
      
      this.processInitialTeamSync(receivedMessages);
      this.simulationTick++;
      
      if (this.simulationTick > 3000 && this.currentOperationMode == OP_MODE_STARTUP) {
         this.currentOperationMode = OP_MODE_MOVING;
      }

      if (this.continuousCounter > 460) {
         this.continuousCounter = 0;
      }

      if (this.getHealth() == 0.0) {
         this.currentOperationMode = OP_MODE_DESTROYED;
      }

      this.synchronizePosition();
      this.outputTelemetry();
      this.executeMainBehavior(receivedMessages);
   }

   private void processInitialTeamSync(ArrayList<String> messages) {
      if (this.simulationTick == 0 && this.myIdentifier == BOT_ID_MAIN_B) {
         for (String message : messages) {
            String[] components = message.split(":");
            if (components.length > 0 && Integer.parseInt(components[0]) == 663121) {
               this.isWestTeam = false;
               this.myPositionX = 2800.0;
               this.myPositionY = 1000.0;
               break;
            }
         }
      }
   }

   private void synchronizePosition() {
      if (this.flagMovingForward) {
         double currentOrientation = this.getOrientationNormalized();
         this.myPositionX += Math.cos(currentOrientation);
         this.myPositionY += Math.sin(currentOrientation);
         this.constrainToArena();
         this.flagMovingForward = false;
      }

      if (this.flagMovingBackward) {
         double currentOrientation = this.getOrientationNormalized();
         this.myPositionX -= Math.cos(currentOrientation);
         this.myPositionY -= Math.sin(currentOrientation);
         this.constrainToArena();
         this.flagMovingBackward = false;
      }
   }

   private void constrainToArena() {
      if (this.myPositionX < 0.0) this.myPositionX = 0.0;
      if (this.myPositionX > 3000.0) this.myPositionX = 3000.0;
      if (this.myPositionY < 0.0) this.myPositionY = 0.0;
      if (this.myPositionY > 2000.0) this.myPositionY = 2000.0;
   }

   private void outputTelemetry() {
      if (this.myIdentifier == BOT_ID_MAIN_A && this.currentOperationMode != OP_MODE_DESTROYED) {
         int orientationDegrees = (int)(this.getOrientationNormalized() * 180.0 / Math.PI);
         this.sendLogMessage("[Unit-A] Position: (" + (int)this.myPositionX + "," + (int)this.myPositionY + ") | Heading: " + orientationDegrees + "° | Mode: " + this.currentOperationMode);
      }

      if (this.myIdentifier == BOT_ID_MAIN_B && this.currentOperationMode != OP_MODE_DESTROYED) {
         int orientationDegrees = (int)(this.getOrientationNormalized() * 180.0 / Math.PI);
         this.sendLogMessage("[Unit-B] Position: (" + (int)this.myPositionX + "," + (int)this.myPositionY + ") | Heading: " + orientationDegrees + "° | Mode: " + this.currentOperationMode);
      }

      if (this.myIdentifier == BOT_ID_MAIN_C && this.currentOperationMode != OP_MODE_DESTROYED) {
         int orientationDegrees = (int)(this.getOrientationNormalized() * 180.0 / Math.PI);
         this.sendLogMessage("[Unit-C] Position: (" + (int)this.myPositionX + "," + (int)this.myPositionY + ") | Heading: " + orientationDegrees + "° | Mode: " + this.currentOperationMode);
      }

      if (this.engagementActive) {
         this.continuousCounter++;
         this.sendLogMessage(">> ENGAGING TARGET <<");
      }
   }

   private void executeMainBehavior(ArrayList<String> messages) {
      this.detectedHostiles.clear();
      
      for (String message : messages) {
         // Handle new format (pipe-separated) messages
         if (message.contains("|")) {
            this.interpretMessage(message);
         } else {
            // Handle old format (colon-separated) messages
            String[] components = message.split(":");
            if (components.length > 2) {
               try {
                  int messageRecipient = Integer.parseInt(components[1]);
                  if (messageRecipient == this.myIdentifier || messageRecipient == BROADCAST_TEAM_ID) {
                     this.interpretMessage(message);
                  }
               } catch (NumberFormatException ignored) {
                  // Skip malformed messages
               }
            }
         }
      }
      
      this.broadcastMyPosition();
      this.scanAndReact();
   }

   private void broadcastMyPosition() {
      String positionMessage = this.myIdentifier + ":" + BROADCAST_TEAM_ID + ":" + MSG_POSITION_SYNC + ":" + 
                   this.myPositionX + ":" + this.myPositionY + ":" + this.getHeading() + ":" + MSG_TERMINATOR;
      this.broadcast(positionMessage);
   }

   private void scanAndReact() {
      boolean immediateThreat = false;
      
      for (IRadarResult contact : this.detectRadar()) {
         if (contact.getObjectType() == characteristics.IRadarResult.Types.OpponentMainBot || 
             contact.getObjectType() == characteristics.IRadarResult.Types.OpponentSecondaryBot) {
            double hostileX = this.myPositionX + contact.getObjectDistance() * Math.cos(contact.getObjectDirection());
            double hostileY = this.myPositionY + contact.getObjectDistance() * Math.sin(contact.getObjectDirection());
            
            double hostileClassification = (contact.getObjectType() == characteristics.IRadarResult.Types.OpponentMainBot) ? 
                              ENEMY_TYPE_PRIMARY : ENEMY_TYPE_SECONDARY;
            
            String enemyAlert = this.myIdentifier + ":" + BROADCAST_TEAM_ID + ":" + MSG_ENEMY_SPOTTED + ":" + 
                        hostileClassification + ":" + hostileX + ":" + hostileY + ":" + MSG_TERMINATOR;
            this.broadcast(enemyAlert);
         }
         
         if (contact.getObjectDistance() < 120.0 && contact.getObjectType() != characteristics.IRadarResult.Types.BULLET && 
             this.currentOperationMode == OP_MODE_MOVING) {
            immediateThreat = true;
         }
      }
      
      if (immediateThreat) {
         this.currentOperationMode = OP_MODE_FALLBACK;
         this.fallbackInitiatedTick = this.simulationTick;
         return;
      }
      
      if (this.engagementActive && !this.seekingFireAngle) {
         this.prioritizeTarget();
      }

      this.handleArenaLimits();
      
      if (this.currentOperationMode == OP_MODE_STARTUP) {
         this.runStartupBehavior();
         return;
      }

      if (this.engagementActive && this.canInitiateFire() && this.verifyFireSafety(this.lockedTargetX, this.lockedTargetY)) {
         this.engageTarget(this.lockedTargetX, this.lockedTargetY);
         this.previousShotTick = this.simulationTick;
      return;
    }

      this.evaluateHuntingMode();
      this.executeHuntingBehavior();
      this.processBehaviorStateMachine();
      
      if (this.currentOperationMode == OP_MODE_DESTROYED) {
         return;
      }
   }

   private void interpretMessage(String message) {
      // Support both old format (colon-separated) and new format (pipe-separated)
      if (message.contains("|")) {
         // New format from MagicSecondary: "POS|id|tick|x|y|heading" or "ENEMY|id|tick|x|y|radius"
         this.parseNewFormatMessage(message);
      } else {
         // Old format: "id:teamId:msgType:data..."
         String[] components = message.split(":");
         if (components.length < 6) return;
         
         int messageCategory = Integer.parseInt(components[2]);
         
         if (messageCategory == MSG_ENEMY_SPOTTED) {
            this.registerHostileContact(components);
         } else if (messageCategory == MSG_POSITION_SYNC) {
            this.updateTeamMemberPosition(components);
         }
      }
   }

   private void parseNewFormatMessage(String message) {
      try {
         String[] parts = message.split("\\|");
         if (parts.length < 4) return;
         
         String msgType = parts[0];
         
         if ("ENEMY".equals(msgType) && parts.length >= 6) {
            // ENEMY|id|tick|x|y|radius
            double x = Double.parseDouble(parts[3]);
            double y = Double.parseDouble(parts[4]);
            
            boolean alreadyKnown = false;
            for (ArrayList<Double> hostile : this.detectedHostiles) {
               if (Math.abs(x - hostile.get(1)) <= POSITION_MATCH_TOLERANCE && 
                   Math.abs(y - hostile.get(2)) <= POSITION_MATCH_TOLERANCE) {
                  alreadyKnown = true;
                  break;
               }
            }
            
            if (!alreadyKnown) {
               ArrayList<Double> newHostile = new ArrayList<>(3);
               newHostile.add(ENEMY_TYPE_PRIMARY);  // Default type
               newHostile.add(x);
               newHostile.add(y);
               this.detectedHostiles.add(newHostile);
            }
            
            this.engagementActive = true;
         } else if ("POS".equals(msgType) && parts.length >= 6) {
            // POS|id|tick|x|y|heading
            String senderId = parts[1];
            // Extract numeric ID from string like "SCOUT_XXXX" or "MAIN_XXXX"
            int numericId = senderId.hashCode();  // Use hashCode as numeric ID
            
            ArrayList<Double> memberState = new ArrayList<>(3);
            memberState.add(Double.parseDouble(parts[3]));  // x
            memberState.add(Double.parseDouble(parts[4]));  // y
            memberState.add(Double.parseDouble(parts[5]));  // heading
            this.teamMemberStates.put(numericId, memberState);
         }
      } catch (Throwable ignored) {
         // Ignore malformed messages
      }
   }

   private void registerHostileContact(String[] components) {
      double x = Double.parseDouble(components[4]);
      double y = Double.parseDouble(components[5]);
      
      boolean alreadyKnown = false;
      for (ArrayList<Double> hostile : this.detectedHostiles) {
         if (Math.abs(x - hostile.get(1)) <= POSITION_MATCH_TOLERANCE && 
             Math.abs(y - hostile.get(2)) <= POSITION_MATCH_TOLERANCE) {
            alreadyKnown = true;
            break;
         }
      }
      
      if (!alreadyKnown) {
         ArrayList<Double> newHostile = new ArrayList<>(3);
         newHostile.add(Double.parseDouble(components[3]));
         newHostile.add(x);
         newHostile.add(y);
         this.detectedHostiles.add(newHostile);
      }
      
      this.engagementActive = true;
   }

   private void updateTeamMemberPosition(String[] components) {
      int senderId = Integer.parseInt(components[0]);
      ArrayList<Double> memberState = new ArrayList<>(3);
      memberState.add(Double.parseDouble(components[3]));
      memberState.add(Double.parseDouble(components[4]));
      memberState.add(Double.parseDouble(components[5]));
      this.teamMemberStates.put(senderId, memberState);
   }

   private void handleArenaLimits() {
      if (this.myPositionX <= 50.0) {
         this.currentOperationMode = this.verifyOrientation(0.0) ? OP_MODE_MOVING : OP_MODE_ORIENT_EAST;
         return;
      }

      if (this.myPositionX >= 2950.0) {
         this.currentOperationMode = this.verifyOrientation(Math.PI) ? OP_MODE_MOVING : OP_MODE_ORIENT_WEST;
         return;
      }

      if (this.myPositionY <= 50.0) {
         this.currentOperationMode = this.verifyOrientation(1.5707963267948966) ? OP_MODE_MOVING : OP_MODE_ORIENT_SOUTH;
         return;
      }

      if (this.myPositionY >= 1950.0) {
         if (this.verifyOrientation(-1.5707963267948966)) {
            this.currentOperationMode = OP_MODE_MOVING;
         } else {
            this.currentOperationMode = OP_MODE_ORIENT_NORTH;
         }
      }
   }

   private void runStartupBehavior() {
      if (this.simulationTick > 100 && this.canInitiateFire()) {
         if (this.engagementActive && this.verifyFireSafety(this.lockedTargetX, this.lockedTargetY)) {
            this.engageTarget(this.lockedTargetX, this.lockedTargetY);
            this.previousShotTick = this.simulationTick;
         } else {
            this.fire(this.getOrientationNormalized());
            this.previousShotTick = this.simulationTick;
         }
      } else {
         this.executeForwardMovement();
      }
   }

   private void evaluateHuntingMode() {
      if (!this.engagementActive && this.simulationTick > 6000 && !this.detectedHostiles.isEmpty() && 
          this.currentOperationMode != OP_MODE_HUNTING) {
         ArrayList<Double> priorityHostile = this.detectedHostiles.get(0);
         double targetX = priorityHostile.get(1);
         double targetY = priorityHostile.get(2);
         double deltaX = Math.abs(targetX - this.myPositionX);
         double deltaY = Math.abs(targetY - this.myPositionY);
         
         if (deltaX > deltaY || deltaY < 200.0) {
            this.currentOperationMode = OP_MODE_HUNTING;
            this.huntingDirection = "x";
         } else if (deltaX > 200.0) {
            this.currentOperationMode = OP_MODE_HUNTING;
            this.huntingDirection = "y";
         }
      }
   }

   private void executeHuntingBehavior() {
      if (this.currentOperationMode == OP_MODE_HUNTING && !this.engagementActive) {
         if (this.detectedHostiles.isEmpty()) {
            this.currentOperationMode = OP_MODE_MOVING;
            this.huntingDirection = "";
            return;
         }
         
         if ("x".equals(this.huntingDirection)) {
            this.huntAlongXAxis();
         } else {
            this.huntAlongYAxis();
         }
      } else if (this.currentOperationMode == OP_MODE_HUNTING && this.engagementActive) {
         this.currentOperationMode = OP_MODE_MOVING;
         this.executeForwardMovement();
         this.huntingDirection = "";
      }
   }

   private void huntAlongXAxis() {
      ArrayList<Double> hostile = this.detectedHostiles.get(0);
      double targetX = hostile.get(1);
      double separation = Math.abs(targetX - this.myPositionX);
      
      if (separation < 200.0) {
         this.huntingDirection = "";
         this.currentOperationMode = OP_MODE_MOVING;
         return;
      }
      
      if (targetX < this.myPositionX) {
         if (this.verifyOrientation(Math.PI)) {
            this.executeForwardMovement();
         } else {
            this.rotateTowardAngle(Math.PI);
         }
      } else {
         if (this.verifyOrientation(0.0)) {
            this.executeForwardMovement();
         } else {
            this.rotateTowardAngle(0.0);
         }
      }
   }

   private void huntAlongYAxis() {
      ArrayList<Double> hostile = this.detectedHostiles.get(0);
      double targetY = hostile.get(2);
      double separation = Math.abs(targetY - this.myPositionY);
      
      if (separation < 200.0) {
         this.huntingDirection = "";
         this.currentOperationMode = OP_MODE_MOVING;
         return;
      }
      
      if (targetY < this.myPositionY) {
         if (this.verifyOrientation(-1.5707963267948966)) {
            this.executeForwardMovement();
         } else {
            this.rotateTowardAngle(-1.5707963267948966);
         }
      } else {
         if (this.verifyOrientation(1.5707963267948966)) {
            this.executeForwardMovement();
         } else {
            this.rotateTowardAngle(1.5707963267948966);
         }
      }
   }

   private void rotateTowardAngle(double desiredAngle) {
      double currentOrientation = this.getOrientationNormalized();
      
      if (desiredAngle == 0.0 || desiredAngle == Math.PI) {
         if (currentOrientation < Math.PI && currentOrientation > 0.0) {
            this.stepTurn(desiredAngle == 0.0 ? Direction.LEFT : Direction.RIGHT);
         } else {
            this.stepTurn(desiredAngle == 0.0 ? Direction.RIGHT : Direction.LEFT);
         }
      } else {
         if (!(currentOrientation < 1.5707963267948966) && !(currentOrientation > 4.71238898038469)) {
            this.stepTurn(desiredAngle < 0.0 ? Direction.RIGHT : Direction.LEFT);
         } else {
            this.stepTurn(desiredAngle < 0.0 ? Direction.LEFT : Direction.RIGHT);
         }
      }
   }

   private void processBehaviorStateMachine() {
      if (this.currentOperationMode == OP_MODE_MOVING) {
         this.executeMovementMode();
      } else if (this.currentOperationMode == OP_MODE_STATIC_FIRE) {
         this.executeStaticFireMode();
      } else if (this.currentOperationMode == OP_MODE_MOBILE_FIRE) {
         this.executeMobileFireMode();
      } else if (this.currentOperationMode == OP_MODE_FALLBACK) {
         this.executeFallbackMode();
      } else if (this.currentOperationMode == OP_MODE_TURN_LEFT || this.currentOperationMode == OP_MODE_TURN_RIGHT) {
         this.executeTurnMode();
      } else {
         this.executeOrientationModes();
      }
   }

   private void executeMovementMode() {
      if (this.detectFront().getObjectType() == Types.WALL) {
         this.handleObstacle();
         return;
      }
      
      if (this.canInitiateFire()) {
         for (int attempt = 0; attempt < 10; attempt++) {
            double randomOffset = this.randomGenerator.nextDouble() * Math.PI / 6.0 - 0.2617993877991494;
            double testX = this.myPositionX + 1000.0 * Math.cos(this.getOrientationNormalized() + randomOffset);
            double testY = this.myPositionY + 1000.0 * Math.sin(this.getOrientationNormalized() + randomOffset);
            if (this.verifyFireSafety(testX, testY)) {
               this.engageTarget(testX, testY);
               this.previousShotTick = this.simulationTick;
               return;
            }
         }
      }
      this.executeForwardMovement();
   }

   private void executeStaticFireMode() {
      this.currentOperationMode = OP_MODE_MOVING;
      if (++this.shotCounter % 2 == 0 && this.continuousCounter < 415) {
         this.moveBack();
         double orientation = this.getHeading();
         this.myPositionX -= Math.cos(orientation);
         this.myPositionY -= Math.sin(orientation);
         this.constrainToArena();
      } else {
         this.executeForwardMovement();
      }
   }

   private void executeMobileFireMode() {
      this.currentOperationMode = OP_MODE_MOVING;
      if (++this.shotCounter % 1 == 0) {
         this.moveBack();
         double orientation = this.getHeading();
         this.myPositionX -= Math.cos(orientation);
         this.myPositionY -= Math.sin(orientation);
         this.constrainToArena();
      } else {
         this.executeForwardMovement();
      }
   }

   private void executeFallbackMode() {
      if (this.simulationTick < this.fallbackInitiatedTick + 25) {
         this.executeBackwardMovement();
      } else {
         if (Math.random() < 0.5) {
            this.currentOperationMode = OP_MODE_TURN_LEFT;
            this.targetOrientation = this.getHeading() + -1.5707963267948966;
            this.stepTurn(Direction.LEFT);
         } else {
            this.currentOperationMode = OP_MODE_TURN_RIGHT;
            this.targetOrientation = this.getHeading() + 1.5707963267948966;
            this.stepTurn(Direction.RIGHT);
         }
      }
   }

   private void executeTurnMode() {
      if (this.verifyOrientation(this.targetOrientation)) {
         this.currentOperationMode = OP_MODE_MOVING;
         this.executeForwardMovement();
      } else {
         Direction turnDirection = (this.currentOperationMode == OP_MODE_TURN_LEFT) ? Direction.LEFT : Direction.RIGHT;
         this.stepTurn(turnDirection);
      }
   }

   private void executeOrientationModes() {
      if (this.currentOperationMode == OP_MODE_ORIENT_NORTH) {
         if (this.verifyOrientation(-1.5707963267948966)) {
            this.currentOperationMode = OP_MODE_MOVING;
            this.executeForwardMovement();
         } else {
            this.rotateTowardAngle(-1.5707963267948966);
         }
      } else if (this.currentOperationMode == OP_MODE_ORIENT_SOUTH) {
         if (this.verifyOrientation(1.5707963267948966)) {
            this.currentOperationMode = OP_MODE_MOVING;
            this.executeForwardMovement();
         } else {
            this.rotateTowardAngle(1.5707963267948966);
         }
      } else if (this.currentOperationMode == OP_MODE_ORIENT_EAST) {
         if (this.verifyOrientation(0.0)) {
            this.currentOperationMode = OP_MODE_MOVING;
            this.executeForwardMovement();
         } else {
            this.rotateTowardAngle(0.0);
         }
      } else if (this.currentOperationMode == OP_MODE_ORIENT_WEST) {
         if (this.verifyOrientation(Math.PI)) {
            this.currentOperationMode = OP_MODE_MOVING;
            this.executeForwardMovement();
         } else {
            this.rotateTowardAngle(Math.PI);
         }
      }
   }

   private void handleObstacle() {
      // Immediate action: always turn when facing a wall, don't just stand there
      this.currentOperationMode = OP_MODE_TURN_LEFT;
      this.targetOrientation = this.getHeading() + -1.5707963267948966;
      this.stepTurn(Direction.LEFT);
   }

   private void prioritizeTarget() {
      ArrayList<ArrayList<Double>> viableTargets = new ArrayList<>();
      
      for (ArrayList<Double> hostile : this.detectedHostiles) {
         double range = this.measureDistance(this.myPositionX, this.myPositionY, hostile.get(1), hostile.get(2));
         if (range <= 1000.0) {
            viableTargets.add(hostile);
         }
      }
      
      viableTargets.sort((h1, h2) -> {
         double r1 = this.measureDistance(this.myPositionX, this.myPositionY, h1.get(1), h1.get(2));
         double r2 = this.measureDistance(this.myPositionX, this.myPositionY, h2.get(1), h2.get(2));
         return Double.compare(r1, r2);
      });
      
      for (ArrayList<Double> hostile : viableTargets) {
         if (this.verifyFireSafety(hostile.get(1), hostile.get(2))) {
            this.lockedTargetX = hostile.get(1);
            this.lockedTargetY = hostile.get(2);
            
            double range = this.measureDistance(this.myPositionX, this.myPositionY, this.lockedTargetX, this.lockedTargetY);
            this.currentOperationMode = (range > 600.0) ? OP_MODE_STATIC_FIRE : OP_MODE_MOBILE_FIRE;
            return;
         }
      }
      
      this.engagementActive = false;
   }

   private void engageTarget(double x, double y) {
      double firingAngle;
      if (this.myPositionX <= x) {
         firingAngle = Math.atan((y - this.myPositionY) / (x - this.myPositionX));
      } else {
         firingAngle = Math.PI + Math.atan((y - this.myPositionY) / (x - this.myPositionX));
      }
      this.fire(firingAngle);
   }

   private boolean verifyFireSafety(double x, double y) {
      double trajectorySlope = (y - this.myPositionY) / (x - this.myPositionX);
      double trajectoryIntercept = this.myPositionY - trajectorySlope * this.myPositionX;
      
      for (ArrayList<Double> teammate : this.teamMemberStates.values()) {
         double teammateX = teammate.get(0);
         double teammateY = teammate.get(1);
         
         if (this.measureDistance(this.myPositionX, this.myPositionY, teammateX, teammateY) <= 10.0) {
            continue;
         }
         
         double angleToTeammate = this.calculateBearingTo(teammateX, teammateY);
         double angleToTarget = this.calculateBearingTo(x, y);
         
         if (Math.abs(angleToTeammate - angleToTarget) < 0.2617993877991494 && 
             this.measureDistance(this.myPositionX, this.myPositionY, teammateX, teammateY) < 
             this.measureDistance(this.myPositionX, this.myPositionY, x, y)) {
            return false;
         }
         
         double orientation = this.getHeading();
         if ((orientation == 0.0 && Math.abs(teammateY - this.myPositionY) < 15.0 && teammateX > this.myPositionX) ||
             (orientation == Math.PI && Math.abs(teammateY - this.myPositionY) < 15.0 && teammateX < this.myPositionX) ||
             (orientation == 1.5707963267948966 && Math.abs(teammateX - this.myPositionX) < 15.0 && teammateY > this.myPositionY) ||
             (orientation == -1.5707963267948966 && Math.abs(teammateX - this.myPositionX) < 15.0 && teammateY < this.myPositionY)) {
            return false;
         }
         
         double teammateSlope = Math.tan(teammate.get(2));
         double teammateIntercept = teammateY - teammateSlope * teammateX;
         double crossingX = (trajectoryIntercept - teammateIntercept) / (teammateSlope - trajectorySlope);
         double crossingY = trajectorySlope * crossingX + trajectoryIntercept;
         
         if (this.measureDistance(teammateX, teammateY, crossingX, crossingY) <= TRAJECTORY_CLEARANCE) {
            boolean xWithinRange = (x >= crossingX && crossingX >= this.myPositionX) || 
                             (x <= crossingX && crossingX <= this.myPositionX);
            boolean yWithinRange = (y >= crossingY && crossingY >= this.myPositionY) || 
                             (y <= crossingY && crossingY <= this.myPositionY);
            if (xWithinRange && yWithinRange) {
               return false;
            }
         }
      }
      
      Types obstacleAhead = this.detectFront().getObjectType();
      return obstacleAhead != Types.TeamMainBot && obstacleAhead != Types.TeamSecondaryBot;
   }

   private double measureDistance(double x1, double y1, double x2, double y2) {
      double dx = x2 - x1;
      double dy = y2 - y1;
      return Math.sqrt(dx * dx + dy * dy);
   }

   private double calculateBearingTo(double x, double y) {
      return this.convertAngleToStandard(Math.atan2(y - this.myPositionY, x - this.myPositionX));
   }

   private double convertAngleToStandard(double angle) {
      while (angle < 0.0) {
         angle += 6.283185307179586;
      }
      while (angle >= 6.283185307179586) {
         angle -= 6.283185307179586;
      }
      return angle;
   }

   private double getOrientationNormalized() {
      return this.convertAngleToStandard(this.getHeading());
   }

   private boolean checkAngleEquality(double angle1, double angle2) {
      return Math.abs(this.convertAngleToStandard(angle1) - this.convertAngleToStandard(angle2)) < ANGLE_MATCH_TIGHT;
   }

   private boolean verifyOrientation(double targetAngle) {
      return Math.abs(Math.sin(this.getHeading() - targetAngle)) < ANGLE_MATCH_LOOSE;
   }

   private boolean canInitiateFire() {
      return this.simulationTick > this.previousShotTick + 20;
   }

   private void executeForwardMovement() {
      this.flagMovingForward = true;
      this.move();
   }

   private void executeBackwardMovement() {
      this.flagMovingBackward = true;
      this.moveBack();
   }
}
