package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;

@Configurable
public class BatBotSmartAI extends BatBotAI
{

public long indexerMoveDelay = 0;

    // --- Slot truth helpers ---
    private boolean ballLatched = false;
    private boolean ballLatchedPrev = false;
    private int emptyStableLoops = 0;
    private int ballInLoops = 0;
    private int ballOutLoops = 0;

    // Shoot clearing confirmation window
    private long pendingClearUntilMs = 0;
    private int pendingClearSlot = -1;

    public BatBotSmartAI() {
        super();
    }

    public void setIndexerPosition(int index) {
        telemetry.addData("setIndexerPosition: ",  index);
        if (index == -1) {
            index = 2;
        } else if (index == 3) {
            index = 0;
        }
        if (index >= 0 && index < 3) {
            double indexerServoPositionNew = STEMperFiConstants.INDEXES.get(index);
            if (indexerServoPositionNew != indexerServoPosition) {
                indexerServoPosition = indexerServoPositionNew;
                indexerServo.setPosition(indexerServoPosition);
                indexerTimePressed = now;
                indexerMoveDelay = now + STEMperFiConstants.INTAKE_DURING_INDEXER_MOVE_MS;
                indexerIndex = index;
                indexMoved = true;
            }
        }
    }

    public boolean isIndexerMoving() {
        return now < indexerMoveDelay;
    }

    public void startLoop() {
        super.startLoop();

        // Slot truth loop hooks
        updateBallLatch();
        reconcileCurrentSlotTruth();

        telemetry.addData("index", indexerIndex);
        telemetry.addData("colors", getColorString());

        // --- Laser sensor debug telemetry (binary mode) ---
        if (telemetry != null) {
            telemetry.addData("Laser activeLow", laserActiveLow);

            // Raw digital input states (HIGH=true, LOW=false)
            telemetry.addData("LaserLeft raw",  (laserLeft  != null) ? laserLeft.getState()  : "null");
            telemetry.addData("LaserRight raw", (laserRight != null) ? laserRight.getState() : "null");

            // Interpreted presence using current polarity setting
            telemetry.addData("LaserLeft present",  laserPresent(laserLeft));
            telemetry.addData("LaserRight present", laserPresent(laserRight));
        }

        if (now > indexerMoveDelay) {
            setInderLed();
        }
    }

    public String getColorString() {
        String c0 = indexerContents.get(0) != Color.WHITE ? "1" : "0";
        String c1 = indexerContents.get(1) != Color.WHITE ? "1" : "0";
        String c2 = indexerContents.get(2) != Color.WHITE ? "1" : "0";
        return c0 + c1 + c2;
    }

    long shootPressed = 0;
    long flipperDownDelay = 0;

    long flipperUpDelay = 0;
    public void shoot() {
        if (!intakeOn && now > indexerMoveDelay && !isIndexerEmpty()) {
            if (now > flipperDownDelay && shootPressed > 0) {
                telemetry.addLine("flipper down");
                shootPressed = 0;
                if (isIndexerEmpty()) {
                    if (indexerIndex != 0) {
                        setIndexerPosition(0);
                    }
                    return;
                }
                setIndexerPosition(indexerIndex - 1);
                return;
            }
            if (now > flipperUpDelay && shootPressed > 0) {
                telemetry.addLine("flipper up");
            //    indexerContents.set(indexerIndex, Color.WHITE);
                // Only clear the slot if the ball actually left the sensing area
                if (!isBallIn()) {
                    indexerContents.set(indexerIndex, Color.WHITE);
                } else {
                    telemetry.addLine("WARN: Ball still present - not clearing slot");
                }
                flipperServoPosition = STEMperFiConstants.FLIPPER_INTAKE;
                flipperServo.setPosition(flipperServoPosition);
                return;
            }
            telemetry.addData("shooterTriggerPressed", shooterTriggerPressed);
            telemetry.addData("shootPressed", shootPressed);
            telemetry.addData("isOnTarget", isOnTarget);
            telemetry.addData("lastDetect", lastDetect);

            boolean targetFresh = (now - lastDetect) <=150;
            if (shooterTriggerPressed && shootPressed == 0 && isOnTarget && targetFresh) {
                if (indexerContents.get(indexerIndex) == Color.WHITE) {
                    if (indexerContents.get(2) != Color.WHITE) {
                      setIndexerPosition(2);
                    } else if (indexerContents.get(1) != Color.WHITE) {
                        setIndexerPosition(1);
                    } else {
                        setIndexerPosition(0);
                    }
                    return;
                }
                shootPressed = now;
                flipperUpDelay = now + 300; //Was 500 up and down, 250 likely most aggressive possible
                flipperDownDelay = now + (300 * 2);
                flipperServoPosition = STEMperFiConstants.FLIPPER_SHOOT;
                flipperServo.setPosition(flipperServoPosition);
            }
        } else if (isIndexerEmpty() && indexerIndex != 0) {
            setIndexerPosition(0);
        }
    }

    public void shootNoLock() {
        if (!intakeOn && now > indexerMoveDelay && !isIndexerEmpty()) {
            if (now > flipperDownDelay && shootPressed > 0) {
                telemetry.addLine("flipper down");
                shootPressed = 0;
                if (isIndexerEmpty()) {
                    if (indexerIndex != 0) {
                        setIndexerPosition(0);
                    }
                    return;
                }
                setIndexerPosition(indexerIndex - 1);
                return;
            }
            if (now > flipperUpDelay && shootPressed > 0) {
                telemetry.addLine("flipper up");
            //    indexerContents.set(indexerIndex, Color.WHITE);

                // Only clear the slot if the ball actually left the sensing area
                if (!isBallIn()) {
                    indexerContents.set(indexerIndex, Color.WHITE);
                } else {
                    telemetry.addLine("WARN: Ball still present - not clearing slot");
                }

                flipperServoPosition = STEMperFiConstants.FLIPPER_INTAKE;
                flipperServo.setPosition(flipperServoPosition);
                return;
            }
            telemetry.addData("shooterTriggerPressed", shooterTriggerPressed);
            telemetry.addData("shootPressed", shootPressed);
            telemetry.addData("isOnTarget", isOnTarget);
            telemetry.addData("lastDetect", now);
            if (shooterTriggerPressed && shootPressed == 0) {
                shootPressed = now;
                flipperUpDelay = now + 500;
                flipperDownDelay = now + (500 * 2);
                flipperServoPosition = STEMperFiConstants.FLIPPER_SHOOT;
                flipperServo.setPosition(flipperServoPosition);
            }
        } else if (isIndexerEmpty() && indexerIndex != 0) {
            setIndexerPosition(0);
        }
    }

    @Override
    public void indexer(boolean isInit) {
        // INDEXER
        if (!shooterTriggerPressed && now > indexDelayDueToShooting) {
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                if (isIndexerEmpty() && !isInit) {
                    gamepad2.runRumbleEffect(customRumbleEffect);
                    return;
                }
                setIndexerPosition(1);
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                if (isIndexerEmpty() && !isInit) {
                    gamepad2.runRumbleEffect(customRumbleEffect);
                    return;
                }
                setIndexerPosition(0);
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                if (isIndexerEmpty() && !isInit) {
                    gamepad2.runRumbleEffect(customRumbleEffect);
                    return;
                }
                setIndexerPosition(2);
            }
        }
    }
    private double percentDifference(double newValue, double oldValue) {
        return oldValue != 0 ? 100 * (newValue - oldValue) / oldValue : 0;
    }

    @Override
    public void intake() {
        if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            intakeOn = !intakeOn;
        }

        double leftTrigger = gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double intakePower = 0;
        if (intakeOn || shooterTriggerPressed || now < indexerMoveDelay) {
            intakePower = 1;
        } else if (leftTrigger > 0.2) {
            intakePower = -leftTrigger;
        }
        intakeMotor.setPower(intakePower);

        if (intakePower > 0.5 && now > indexerMoveDelay && intakeOn && !ballLatchedPrev && ballLatched) {
            Color ballColor = Color.RED;
            if (indexerIndex == 1) {
                ballColor = Color.BLUE;
            }
            if (indexerIndex == 2) {
                ballColor = Color.YELLOW;
            }
            indexerContents.set(indexerIndex, ballColor);
            if (isIndexerFull()) {
                setIndexerPosition(2);
            } else {
                setIndexerPosition(nextEmptySlot());
            }
        }
        ballLatchedPrev = ballLatched;
    }

    public void setInderLed() {
        Color currentColor = indexerContents.get(indexerIndex);
        if (currentColor == Color.RED) {
            indexerLed.setPosition(STEMperFiConstants.GB_LED_RED);
        } else if (currentColor == Color.BLUE) {
            indexerLed.setPosition(STEMperFiConstants.GB_LED_BLUE);
        } else if (currentColor == Color.YELLOW) {
            indexerLed.setPosition(STEMperFiConstants.GB_LED_YELLOW);
        } else {
            indexerLed.setPosition(STEMperFiConstants.GB_LED_WHITE);
        }
    }

//    public Color determineColor(){
//        //Evaluate color
//        Color answer = Color.WHITE;
//        //Read the color sensors
//        NormalizedRGBA cs2RgbaNew = cs2.getNormalizedColors();
//        NormalizedRGBA cs3RgbaNew = cs3.getNormalizedColors();
//        double cs2PercentDiffRed = percentDifference(cs2RgbaNew.red, cs2RgbaBase.red);
//        double cs2PercentDiffGreen = percentDifference(cs2RgbaNew.green, cs2RgbaBase.green);
//        double cs2PercentDiffBlue = percentDifference(cs2RgbaNew.blue, cs2RgbaBase.blue);
//        double cs3PercentDiffBlue = percentDifference(cs3RgbaNew.blue, cs3RgbaBase.blue);
//        // Green
//        int greenPoints = 0;
//        if (cs2PercentDiffGreen > cs2PercentDiffBlue && cs2PercentDiffGreen > cs2PercentDiffRed && cs2PercentDiffGreen > 80) {
//            greenPoints++;
//        }
//        if (cs3PercentDiffBlue > 100 && cs3PercentDiffBlue < 160) {
//            greenPoints++;
//        }
//        // purple
//        int purplePoints = 0;
//        if (cs2PercentDiffGreen < cs2PercentDiffBlue && cs2PercentDiffGreen < cs2PercentDiffRed && cs2PercentDiffRed > 80 && cs2PercentDiffBlue > 80) {
//            purplePoints++;
//        }
//        if (cs2PercentDiffBlue > 160) {
//            purplePoints++;
//        }
//        if (greenPoints > 0 || purplePoints > 0) {
//            answer = greenPoints > purplePoints ? Color.GREEN : Color.PURPLE;
//        }
//        return answer;
//    }
//
//    public Color getCurrentColor() {
//        return indexerContents.get(indexerIndex);
//    }
//

    private Color slotColorForIndex(int slot) {
        if (slot == 0) return Color.RED;
        if (slot == 1) return Color.BLUE;
        return Color.YELLOW;
    }

    private void updateBallLatch() {
        boolean present = isBallIn(); // your fused presence check

        if (present) {
            ballInLoops++;
            ballOutLoops = 0;
            if (!ballLatched && ballInLoops >= STEMperFiConstants.INTAKE_CYCLE_COUNT_BALL_IS_IN) {
                ballLatched = true;
                ballInLoops = 0;
            }
        } else {
            ballOutLoops++;
            ballInLoops = 0;
            if (ballLatched && ballOutLoops >= 3) {
                ballLatched = false;
                ballOutLoops = 0;
            }
        }
    }

    private void reconcileCurrentSlotTruth() {
        if (now <= indexerMoveDelay) return;
    //    if (ballLatched && indexerContents.get(indexerIndex) == Color.WHITE) {
    //       indexerContents.set(indexerIndex, slotColorForIndex(indexerIndex));
    //    }
        if (!ballLatchedPrev && ballLatched && indexerContents.get(indexerIndex) == Color.WHITE) {
            indexerContents.set(indexerIndex, slotColorForIndex(indexerIndex));
        }
        // Conservative clear: only clear after stable empty for a few loops
        if (!ballLatched && indexerContents.get(indexerIndex) != Color.WHITE) {
            emptyStableLoops++;
            if (emptyStableLoops >= 5) {
                indexerContents.set(indexerIndex, Color.WHITE);
                emptyStableLoops = 0;
            }
        } else {
            emptyStableLoops = 0;
        }
    }


    public int intakeDetectCycleCount = 0;

    private Boolean cs2PresentSafe() {
        try {
            double d = ((DistanceSensor) cs2).getDistance(DistanceUnit.CM);
            if (Double.isNaN(d) || Double.isInfinite(d) || d <= 0.1) return null; // invalid
            return d < STEMperFiConstants.BALL_DETECTION_DISTANCE_CM;
        } catch (Exception e) {
            return null;
        }
    }

    private Boolean laserPresentSafe(DigitalChannel laser) {
        if (laser == null) return null;
        try {
            return laserPresent(laser); // uses laserActiveLow inversion in BatBotAI
        } catch (Exception e) {
            return null;
        }
    }


    @Override
    public boolean isBallIn() {
        //2 of 3 sensor voting + debounce counter to determine ball intake
        //Uncomment to switch between this and the one below.

        // Don’t trust readings while the indexer is moving/settling
        if (now <= indexerMoveDelay) {
            intakeDetectCycleCount = 0;
            return false;
        }

        Boolean c = cs2PresentSafe();
        Boolean l = laserPresentSafe(laserLeft);
        Boolean r = laserPresentSafe(laserRight);

        int valid = 0;
        int inCnt = 0;

        if (c != null) { valid++; if (c) inCnt++; }
        if (l != null) { valid++; if (l) inCnt++; }
        if (r != null) { valid++; if (r) inCnt++; }

        boolean present;
        if (valid >= 2) {
            present = inCnt >= 2;         // majority vote
        } else if (valid == 1) {
            present = inCnt == 1;         // trust the one valid sensor
        } else {
            present = false;              // no valid data
        }

        // Keep your existing debounce behavior
        if (present) intakeDetectCycleCount++;
        else intakeDetectCycleCount = 0;

        return intakeDetectCycleCount >= STEMperFiConstants.INTAKE_CYCLE_COUNT_BALL_IS_IN;
    }
  /*
    public boolean isBallIn() {
        //Check sensors flexibly and be OK if the laser distance sensors aren't responding
        //Uncomment to switch between this and the one above.

        // Don’t trust presence readings while the indexer is moving/settling
        if (now <= indexerMoveDelay) {
            intakeDetectCycleCount = 0;
            return false;
        }

        boolean cs2Present = false;
        try {
            cs2Present = ((DistanceSensor) cs2).getDistance(DistanceUnit.CM)
                    < STEMperFiConstants.BALL_DETECTION_DISTANCE_CM;
        } catch (Exception ignored) {
            cs2Present = false;
        }

        boolean leftPresent  = laserPresent(laserLeft);
        boolean rightPresent = laserPresent(laserRight);

        // Presence if ANY says present
        boolean present = cs2Present || leftPresent || rightPresent;

        if (present) {
            intakeDetectCycleCount++;
        } else {
            intakeDetectCycleCount = 0;
        }

        return intakeDetectCycleCount >= STEMperFiConstants.INTAKE_CYCLE_COUNT_BALL_IS_IN;


        //       if (((DistanceSensor) cs2).getDistance(DistanceUnit.CM) < STEMperFiConstants.BALL_DETECTION_DISTANCE_CM && intakeOn && now > indexerMoveDelay) {
 //           intakeDetectCycleCount++;
 //       } else {
 //           intakeDetectCycleCount = 0;
 //       }
 //       return intakeDetectCycleCount >= STEMperFiConstants.INTAKE_CYCLE_COUNT_BALL_IS_IN;
    }
*/
    public boolean isIndexerFull() {
        return !indexerContents.contains(Color.WHITE);
    }

    public boolean isIndexerEmpty() {
        return indexerContents.get(0) == Color.WHITE && indexerContents.get(1) == Color.WHITE && indexerContents.get(2) == Color.WHITE;
    }


    public int nextEmptySlot() {
        return indexerContents.indexOf(Color.WHITE);
    }
}
