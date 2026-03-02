package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;


@Configurable
public class BatBotSmart extends BatBot
{
    public BatBotSmart() {
        super();
    }

    public void startLoop() {
        super.startLoop();
    }

    long shootPressed = 0;
    long flipperDownDelay = 0;
    long flipperUpDelay = 0;
    public void shoot() {
        if (!intakeOn && now > indexerMoveDelay && !isIndexerEmpty()) {
            if (now > flipperDownDelay && shootPressed > 0) {
                joinedTelemetry.addLine("flipper down");
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
                joinedTelemetry.addLine("flipper up");
                indexerContents[indexerIndex] = STEMperFiConstants.GB_LED_OFF;
                flipperServoPosition = STEMperFiConstants.FLIPPER_INTAKE;
                flipperServo.setPosition(flipperServoPosition);
                return;
            }
            joinedTelemetry.addData("shooterTriggerPressed", shooterTriggerPressed);
            joinedTelemetry.addData("shootPressed", shootPressed);
            joinedTelemetry.addData("isOnTarget", isOnTarget);
            joinedTelemetry.addData("lastDetect", now);
            if (shooterTriggerPressed && shootPressed == 0 && isOnTarget && lastDetect == now) {
                if (indexerContents[indexerIndex] == STEMperFiConstants.GB_LED_OFF) {
                    if (indexerContents[2] != STEMperFiConstants.GB_LED_OFF) {
                      setIndexerPosition(2);
                    } else if (indexerContents[1] != STEMperFiConstants.GB_LED_OFF) {
                        setIndexerPosition(1);
                    } else {
                        setIndexerPosition(0);
                    }
                    return;
                }
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

    public void shootNoLock() {
        if (!intakeOn && now > indexerMoveDelay && !isIndexerEmpty()) {
            if (now > flipperDownDelay && shootPressed > 0) {
                joinedTelemetry.addLine("flipper down");
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
                joinedTelemetry.addLine("flipper up");
                indexerContents[indexerIndex] = STEMperFiConstants.GB_LED_OFF;
                flipperServoPosition = STEMperFiConstants.FLIPPER_INTAKE;
                flipperServo.setPosition(flipperServoPosition);
                return;
            }
            joinedTelemetry.addData("shooterTriggerPressed", shooterTriggerPressed);
            joinedTelemetry.addData("shootPressed", shootPressed);
            joinedTelemetry.addData("isOnTarget", isOnTarget);
            joinedTelemetry.addData("lastDetect", now);
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
        if (!shooterTriggerPressed && !isIndexerMoving()) {
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                setIndexerPosition(1);
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                setIndexerPosition(0);
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
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

        if (!isIndexerMoving() && intakeOn && isBallIn()) {
            double ballColor = determineColor();
            indexerContents[indexerIndex] = ballColor;
            if (isIndexerFull()) {
                setIndexerPosition(2);
            } else {
                setIndexerPosition(nextEmptySlot());
            }
        }
    }

}
