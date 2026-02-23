package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

import java.util.Arrays;
import java.util.List;

public class STEMperFiConstants {
    public static final int INTAKE_CYCLE_COUNT_BALL_IS_IN = 3;
    public static final double TURRET_ENCODER_COUNTS_PER_REV = 8192;
    public static final double TURRET_SMALL_PULLY_TEETH_PER_REV = 60;
    public static final double TURRET_LAZY_TEETH_PER_REV = 350;
    public static final double TURRET_TICKS_PER_DEGREE = (TURRET_LAZY_TEETH_PER_REV / TURRET_SMALL_PULLY_TEETH_PER_REV) * TURRET_ENCODER_COUNTS_PER_REV / 360;
    public static final int TURRET_MAX_TICKS = (int) (TURRET_TICKS_PER_DEGREE * 90);
    public static final int TURRET_MIN_TICKS = -TURRET_MAX_TICKS;
    public static final double TURRET_MOTOR_POWER_MAX = 0.8;
    public static final double TURRET_MOTOR_POWER_MIN = .3;

    // +- 1% target considered on target
    public static final int TURRET_TARGET_DELTA = (int) (TURRET_MAX_TICKS * 0.01);


    public static final String BLACKBOARD_KEY_ALLIANCE = "BLACKBOARD_KEY_ALLIANCE";
    public static final String BLACKBOARD_KEY_PATTERN = "BLACKBOARD_KEY_PATTERN";
    public static final String ALLIANCE_BLUE = "BLUE";
    public static final String ALLIANCE_RED = "RED";

    public static final String PATTERN_21_GPP = "GPP";
    public static final String PATTERN_22_PGP = "PGP";
    public static final String PATTERN_23_PPG = "PPG";

    public static final double FLIPPER_INTAKE = 0.52;
    public static final double FLIPPER_SHOOT = .25;

    public static final double INDEX_1 = .8;
    public static final double INDEX_2 = .4;
    public static final double INDEX_3 = 0.02;

    public static final List<Double> INDEXES = Arrays.asList(INDEX_1, INDEX_2, INDEX_3);

    public static final double BALL_DETECTION_DISTANCE_CM=7;


    public static final float COLOR_SENSOR_GAIN = 2.0f;

    public static final int[] AUTO_SHOTS_21_GPP = {0, 1, 2};
    public static final int[] AUTO_SHOTS_22_PGP = {1, 0, 2};
    public static final int[] AUTO_SHOTS_23_PPG = {2, 1, 0};

    public static final long INTAKE_DURING_INDEXER_MOVE_MS = 400;

    public static final double GB_LED_OFF = 0;
    public static final double GB_LED_RED = 0.28;
    public static final double GB_LED_ORANGE = 0.333;
    public static final double GB_LED_YELLOW = 0.388;
    public static final double GB_LED_SAGE = 0.444;
    public static final double GB_LED_GREEN = 0.5;
    public static final double GB_LED_AZURE = 0.555;
    public static final double GB_LED_BLUE = 0.611;
    public static final double GB_LED_INDIGO = 0.666;
    public static final double GB_LED_VIOLET = 0.722;
    public static final double GB_LED_WHITE = 1.0;

    public static final int TICKS_PER_REV_6000RPM = 28;

    public static final double SHOOT_RPM_FAR = 1_100;

    public static final double SHOOT_FAR_TICS_PER_SEC = (SHOOT_RPM_FAR / 60) * TICKS_PER_REV_6000RPM;
    public static final double SHOOT_RPS_FAR = SHOOT_RPM_FAR / 60;

    public static final double SHOOT_RELATIVE_POWER_SHORT = .19;

    public static final double HOOD_RELATIVE_ANGLE_SHORT = 1;

    public static final double SHOOT_RELATIVE_POWER_MED = .24;

    public static final double HOOD_RELATIVE_ANGLE_MED = .6;
    public static final double SHOOT_RELATIVE_POWER_MED_AUTO = .24;
    public static final double SHOOT_RELATIVE_POWER_HIGH = 1;

    public static final long SHOOT_DELAY_INDEX_MS = 300;

    public static final int LIMELIGHT_PIPELINE_BLUE = 7;
    public static final int LIMELIGHT_PIPELINE_RED = 6;
    public static final int LIMELIGHT_PIPELINE_AUTO = 5;
    public static final int LED_BRIGHTNESS = 10;

    public static final int LED_NUM = 35;
    public static final PrismAnimations.Solid[] LEFT_PURPLE = new PrismAnimations.Solid[3];
    public static final PrismAnimations.Solid[] LEFT_GREEN = new PrismAnimations.Solid[3];
    public static final PrismAnimations.Solid[] RIGHT_PURPLE = new PrismAnimations.Solid[3];
    public static final PrismAnimations.Solid[] RIGHT_GREEN = new PrismAnimations.Solid[3];
    public static PrismAnimations.Solid getAnimationSolid(Color color, int startIndex, int stopIndex, int brightness) {
        PrismAnimations.Solid result = new PrismAnimations.Solid(color);
        result.setStartIndex(startIndex);
        result.setStopIndex(stopIndex);
        result.setBrightness(brightness);
        return result;
    }

    static {
        for (int i = 0; i < 3; i++) {
            int startIndex = i * 4;
            int endIndex = startIndex + 3;
            LEFT_PURPLE[i] = getAnimationSolid(Color.PURPLE, startIndex, endIndex, LED_BRIGHTNESS);
            LEFT_GREEN[i] = getAnimationSolid(Color.GREEN, startIndex, endIndex, LED_BRIGHTNESS);
        }
        for (int i = 0; i < 3; i++) {
            int offset = 12;
            int startIndex = offset + (i * 4);
            int endIndex = startIndex + 3;
            RIGHT_PURPLE[i] = getAnimationSolid(Color.PURPLE, startIndex, endIndex, LED_BRIGHTNESS);
            RIGHT_GREEN[i] = getAnimationSolid(Color.GREEN, startIndex, endIndex, LED_BRIGHTNESS);
        }
    }



}
