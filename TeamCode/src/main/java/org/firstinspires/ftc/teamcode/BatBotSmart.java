package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;


@Configurable
public class BatBotSmart extends BatBot
{
    public BatBotSmart() {
        super();
    }

    private int healPresentLoops = 0;
    private int healEmptyLoops = 0;

    long shootPressed = 0;
    long flipperDownDelay = 0;
    long flipperUpDelay = 0;

    public boolean isShooting() {
      return isAutoShooterTriggerPressed || shootPressed > 0;
    }

    public boolean canShoot()  {
        return isShooterTriggerPressed && !isShooting() && (isLocked() || !isLockRequiredToShoot) && shooterSpeed > 0;
    }

    public void shoot() {
        if (!intakeOn && !isIndexerMoving()) {
            if (now > flipperDownDelay && isShooting()) {
                joinedTelemetry.addLine("flipper finished moving down down stop shooting");
                shootPressed = 0;
                indexerContents[indexerIndex] = STEMperFiConstants.GB_LED_OFF;
                setIndexerPosition(indexerIndex + 1);
                return;
            }
            if (now > flipperUpDelay && isShooting()) {
                joinedTelemetry.addLine("flipper finished moving up");
                flipperServoPosition = STEMperFiConstants.FLIPPER_INTAKE;
                flipperServo.setPosition(flipperServoPosition);
                return;
            }
            joinedTelemetry.addData("shooterTriggerPressed", isShooterTriggerPressed);
            joinedTelemetry.addData("shootPressed", shootPressed);
            joinedTelemetry.addData("isOnTarget", isTurretStopped);
            joinedTelemetry.addData("lastDetect", now);
            if (canShoot()) {
                shootPressed = now;
                flipperUpDelay = now + 500;
                flipperDownDelay = now + (500 * 2);
                flipperServoPosition = STEMperFiConstants.FLIPPER_SHOOT;
                flipperServo.setPosition(flipperServoPosition);
            }
        }
    }

    @Override
    public void intake() {
        if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            intakeOn = !intakeOn;
        }
        double leftTrigger = gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double intakePower = 0;
        if (intakeOn || isShooting() || isIndexerMoving()) {
            intakePower = 1;
        } else if (leftTrigger > 0.2) {
            intakePower = -leftTrigger;
        }
        intakeMotor.setPower(intakePower);

        joinedTelemetry.addData("isIndexterFull", isIndexerFull());
        if (!isIndexerMoving() && intakeOn && isBallIn() && !isIndexerFull()) {
            double ballColor = determineColor();
            indexerContents[indexerIndex] = ballColor;
            setIndexerPosition(indexerIndex + 1);
        }
    }
    public void autoHealSlotState() {
        joinedTelemetry.addData("autoHealSlotState ", true);
        if (!isShooting() && !isIndexerMoving()) {
            if (isBallIn()) {
                joinedTelemetry.addData("ball IN  index: ", indexerIndex);
                double ballColor = determineColor();
                indexerContents[indexerIndex] = ballColor;
            } else {
                joinedTelemetry.addData("ball OUT index: ", indexerIndex);
                indexerContents[indexerIndex] = STEMperFiConstants.GB_LED_OFF;
            }
        }
        // Skip heal while system is busy
//        if (intakeOn || shootPressed > 0 || now <= indexerMoveDelay) return;
//
//        // Physical truth (already debounced inside isBallIn())
//        boolean present = isBallIn();
//
//        // Slot state truth (GB_LED_OFF == empty)
//        boolean stateEmpty = (indexerContents[indexerIndex] == STEMperFiConstants.GB_LED_OFF);
//
//        // Case A: ball is physically present but slot is marked empty -> restore after a few loops
//        if (present && stateEmpty) {
//            healPresentLoops++;
//            healEmptyLoops = 0;
//
//            if (healPresentLoops >= 3) {  // minimal bounce check
//                double ballColor = determineColor();
//                indexerContents[indexerIndex] = ballColor;
//
//                joinedTelemetry.addLine("AUTOHEAL: restored slot state (present but empty)");
//                healPresentLoops = 0;
//            }
//            return;
//        }
//
//        // Case B: ball is physically absent but slot is marked filled -> clear after a few loops
//        if (!present && !stateEmpty) {
//            healEmptyLoops++;
//            healPresentLoops = 0;
//
//            if (healEmptyLoops >= 5) {	// slightly more conservative for clearing
//                indexerContents[indexerIndex] = STEMperFiConstants.GB_LED_OFF;
//
//                joinedTelemetry.addLine("AUTOHEAL: cleared slot state (empty confirmed)");
//                healEmptyLoops = 0;
//            }
//            return;
//        }
//
//        // No mismatch -> reset counters
//        healPresentLoops = 0;
//        healEmptyLoops = 0;
    }

}
