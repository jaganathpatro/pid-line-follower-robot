// ================================================================
//  LINE FOLLOWER v16 — 5-CHANNEL
//  LEFT PRIORITY + SHARP TURN FIX + T-JUNCTION ADVANCE
//  + WIDER TRACK WOBBLE FIX
//
//  CHANGES IN v16 vs v15:
//
//  1. SENSOR WEIGHTS SCALED DOWN for wider chassis:
//     W[] = {-1600, -800, 0, 800, 1600}
//     Keeps error magnitude calibrated after physical width increase.
//
//  2. ZONE 0 PID SOFTENED (straight-line zone only):
//     KP[0]: 0.11 → 0.08
//     KD[0]: 0.90 → 0.65
//
//  3. DERIVATIVE FILTER HEAVIER:
//     ALPHA: 0.40 → 0.58  (kills high-frequency jitter)
//
//  4. CORRECTION CEILING TIGHTENED on straight:
//     CORR_LIMIT[0]: 70 → 48
//
//  5. INTEGRAL BLEED ZONE WIDENED:
//     absErr < 150 → absErr < 220, decay 0.75 → 0.65
//
//  6. ZERO_BAND WIDENED:
//     15 → 22  (stops twitchy motor response near center)
//
//  PREVIOUS CHANGES (from v15, unchanged):
//
//  LEFT PRIORITY (3 changes):
//     a) Block ② (both sides lit) → commits LEFT turn instead of right
//     b) JUNCTION_SEQ → all J_LEFT
//     c) executeJunctionAction() → J_LEFT is the primary action,
//        falls back to right only if no left branch exists
//
//  SHARP TURN FIX:
//     - PIVOT_*_REV = -105 / -90  (aggressive reverse for crisp 90°)
//     - TURN90_MIN_MS = 130 ms
//     - Hard-curve pattern entries use negative inner-wheel (braking)
//     - SPD_SHARP = 42
//     - CORR_LIMIT[2] = 230
//
//  JUNCTION ADVANCE STATE (ST_JUNCTION_ADV):
//     - Drives straight JUNCTION_ADVANCE_MS before executing turn
//       so the axle centres over the intersection.
//
//  RIGHT PRIORITY SCOPE RULE:
//     - Priority ONLY fires in block ② when lCnt>=1 AND rCnt>=1
//       (both sides simultaneously lit).
//     - Single-side turns follow the line naturally — no override.
//
//  SENSOR LAYOUT (under robot, left to right):
//  Index:  0      1      2      3      4
//  Weight:-1600  -800    0    +800  +1600
//
//  lCnt = sv[0]+sv[1]   rCnt = sv[3]+sv[4]   center = sv[2]
//  JUNCTION threshold: 4+ sensors lit
// ================================================================

// ─── HARDWARE ────────────────────────────────────────────────────
#define LINE_DETECTED  LOW
#define MIN_PWM        120
#define MAX_PWW        255
#define ZERO_BAND      22       // v16: widened from 15 → less twitching at center

// ─── MOTOR PINS ──────────────────────────────────────────────────
const int rPwm = 15;
const int rF   = 2;
const int rB   = 18;
const int lPwm = 14;
const int lF   = 12;
const int lB   = 13;

// ─── SENSOR PINS ─────────────────────────────────────────────────
const int sensorPins[5] = {34, 35, 32, 33, 25};

// ─── SENSOR WEIGHTS ──────────────────────────────────────────────
// v16: scaled down from ±2000/±1000 to ±1600/±800
// Keeps error magnitude proportional after track width increase.
const int W[5] = {-1600, -800, 0, 800, 1600};

// ================================================================
//  JUNCTION SEQUENCE TABLE  — all J_LEFT
// ================================================================
enum JunctionAction { J_STRAIGHT, J_LEFT, J_RIGHT, J_UTURN, J_STOP };

const JunctionAction JUNCTION_SEQ[] = {
    J_LEFT,    // 0
    J_LEFT,    // 1
    J_LEFT,    // 2
    J_LEFT,    // 3
    J_LEFT,    // 4
    J_STOP,    // 5
};
const int JUNCTION_SEQ_LEN = sizeof(JUNCTION_SEQ) / sizeof(JunctionAction);
int           junctionCount    = 0;
unsigned long lastJunctionTime = 0;
#define JUNCTION_DEBOUNCE_MS  300

// ================================================================
//  ROBOT STATE MACHINE
// ================================================================
enum RobotState {
    ST_FOLLOW,
    ST_TURN_L,
    ST_TURN_R,
    ST_ARC_L,
    ST_ARC_R,
    ST_UTURN,
    ST_JUNCTION,        // brief straight-hold after J_STRAIGHT
    ST_JUNCTION_ADV,    // advance to intersection centre before turning
    ST_LOST,
    ST_STOP
};
RobotState     robotState    = ST_FOLLOW;
JunctionAction pendingAction = J_STRAIGHT;

// ================================================================
//  TURN CONFIRMATION COUNTERS
// ================================================================
#define TURN90_CONFIRM      4
#define TURN45_CONFIRM      3
#define UTURN_CONFIRM       5
#define BOTH_SIDES_CONFIRM  3   // frames both sides must be lit before priority fires

int confirm90L    = 0, confirm90R   = 0;
int confirm45L    = 0, confirm45R   = 0;
int confirmUT     = 0;
int confirmBothL  = 0;   // left-priority counter (both sides lit)

// ================================================================
//  TIMING
// ================================================================
#define TURN90_MIN_MS        130   // extended so 90° pivot fully completes
#define TURN45_MIN_MS         65
#define UTURN_MIN_MS         220
#define JUNCTION_HOLD_MS      70
#define POST_TURN_COOLDOWN   200
#define JUNCTION_ADVANCE_MS   80   // ms to drive straight into intersection

unsigned long turnEnteredAt     = 0;
unsigned long turnExitedAt      = 0;
unsigned long junctionEnteredAt = 0;
unsigned long junctionAdvAt     = 0;

// ================================================================
//  SPEEDS
// ================================================================
#define SPD_STRAIGHT   144
#define SPD_CURVE       90
#define SPD_SHARP       42
#define SPD_APPROACH    98
#define SPD_JUNCTION    90
#define SPD_ADV         85

// ── 90° pivot ────────────────────────────────────────────────────
#define PIVOT_L_FWD    130
#define PIVOT_L_REV   -105
#define PIVOT_R_FWD    120
#define PIVOT_R_REV    -90

// ── 45° arc ──────────────────────────────────────────────────────
#define ARC_L_IN       -55
#define ARC_L_OUT      118
#define ARC_R_IN       -55
#define ARC_R_OUT      118

// ── U-turn ───────────────────────────────────────────────────────
#define UTURN_L_FWD    115
#define UTURN_L_REV    -78

// ================================================================
//  PID GAINS
//  v16: KP[0] 0.11→0.08, KD[0] 0.90→0.65 (softer on straight)
//       Curve and sharp zones unchanged.
// ================================================================
const float KP[3] = { 0.08f,  0.30f,  0.55f };
const float KD[3] = { 0.65f,  2.80f,  5.50f };
#define KI_GAIN      0.00009f
#define INTG_LIMIT   1500

// ================================================================
//  ZONE THRESHOLDS & CORRECTION LIMITS
//  v16: CORR_LIMIT[0] 70→48 (tighter ceiling on straight)
//       Curve and sharp limits unchanged.
// ================================================================
#define ZONE_CURVE    300
#define ZONE_SHARP    800
const int CORR_LIMIT[3] = { 48, 120, 230 };

// ================================================================
//  DERIVATIVE FILTER
//  v16: ALPHA 0.40→0.58 (heavier smoothing, kills jitter)
// ================================================================
#define ALPHA   0.58f

// ================================================================
//  BITMASK PATTERN TABLE
//  Hard-turn entries use negative inner-wheel values for braking.
// ================================================================
struct PatternEntry { uint8_t mask; int lSpd; int rSpd; };

const PatternEntry PATTERN_TABLE[] = {
    { 0x04, SPD_STRAIGHT, SPD_STRAIGHT },  // center only
    { 0x0E, SPD_STRAIGHT, SPD_STRAIGHT },  // s1,s2,s3 wide center
    { 0x0C,  124, 147 },                   // s1,s2  slight left
    { 0x06,  147, 124 },                   // s2,s3  slight right
    { 0x18,   88, 140 },                   // s0,s1  left
    { 0x03,  140,  88 },                   // s3,s4  right
    { 0x1C,  -20, 138 },                   // s0,s1,s2 hard left  — inner brakes
    { 0x07,  138, -20 },                   // s2,s3,s4 hard right — inner brakes
    { 0x08,  128, 148 },                   // s1 only  lean left
    { 0x02,  148, 128 },                   // s3 only  lean right
    { 0x10,  -30, 138 },                   // s0 only  far left   — inner brakes
    { 0x01,  138, -30 },                   // s4 only  far right  — inner brakes
    { 0x14,  -25, 138 },                   // s0,s2 split-left    — inner brakes
    { 0x05,  138, -25 },                   // s2,s4 split-right   — inner brakes
};
const int PATTERN_COUNT = sizeof(PATTERN_TABLE) / sizeof(PatternEntry);

// ================================================================
//  RECOVERY
// ================================================================
#define JUNCTION_SENSORS  4
#define LOST_SPIN_SPD   173

int recHitCount = 0;
#define REC_HIT_NEEDED  3

// ================================================================
//  PID STATE
// ================================================================
int   lastErr = 0;
float filtD   = 0.0f;
float intgAcc = 0.0f;

// ================================================================
//  HISTORY RING BUFFER
// ================================================================
#define HIST 12
int  eBuf[HIST];
int  hIdx  = 0;
bool hFull = false;

void pushErr(int e) {
    eBuf[hIdx] = e;
    hIdx = (hIdx + 1) % HIST;
    if (!hFull && hIdx == 0) hFull = true;
}

void clearHistory() {
    for (int i = 0; i < HIST; i++) eBuf[i] = 0;
    hIdx = 0; hFull = false;
}

void resetAllConfirm() {
    confirm90L  = 0; confirm90R  = 0;
    confirm45L  = 0; confirm45R  = 0;
    confirmUT   = 0; confirmBothL = 0;
}

// ================================================================
//  LINE LOSS STATE
// ================================================================
bool          linePresent = false;
unsigned long lostTime    = 0;
int           lastSide    = 0;

// ================================================================
//  MOTOR DRIVER
// ================================================================
void driveMotor(int fPin, int bPin, int pwmPin, int val) {
    val = constrain(val, -MAX_PWW, MAX_PWW);
    if (abs(val) < ZERO_BAND) {
        digitalWrite(fPin, LOW); digitalWrite(bPin, LOW);
        analogWrite(pwmPin, 0);
    } else if (val > 0) {
        digitalWrite(fPin, HIGH); digitalWrite(bPin, LOW);
        analogWrite(pwmPin, max(val, MIN_PWM));
    } else {
        digitalWrite(fPin, LOW); digitalWrite(bPin, HIGH);
        analogWrite(pwmPin, max(-val, MIN_PWM));
    }
}

void go(int L, int R) {
    driveMotor(lF, lB, lPwm, L);
    driveMotor(rF, rB, rPwm, R);
}

void stopMotors() {
    digitalWrite(lF, LOW); digitalWrite(lB, LOW); analogWrite(lPwm, 0);
    digitalWrite(rF, LOW); digitalWrite(rB, LOW); analogWrite(rPwm, 0);
}

void syncPID(int error) {
    int rd = constrain(error - lastErr, -1000, 1000);
    filtD   = ALPHA * filtD + (1.0f - ALPHA) * rd;
    lastErr = error;
}

// ================================================================
//  BITMASK HELPERS
// ================================================================
uint8_t buildMask(int* sv) {
    uint8_t m = 0;
    for (int i = 0; i < 5; i++)
        if (sv[i]) m |= (1 << (4 - i));
    return m;
}

int lookupPattern(uint8_t mask) {
    for (int i = 0; i < PATTERN_COUNT; i++)
        if (PATTERN_TABLE[i].mask == mask) return i;
    return -1;
}

// ================================================================
//  SINGLE-SIDE TURN DETECTORS
//  These only fire when ONE side is lit — no priority conflict.
// ================================================================
bool is90Left (int* sv, int lCnt, int rCnt) { return (rCnt==0) && sv[0] && (lCnt>=2); }
bool is90Right(int* sv, int lCnt, int rCnt) { return (lCnt==0) && sv[4] && (rCnt>=2); }

bool is45Left(int* sv, int lCnt, int rCnt) {
    if (rCnt > 0 || is90Left(sv,lCnt,rCnt)) return false;
    return (lCnt==1) && (sv[0] || sv[1]);
}
bool is45Right(int* sv, int lCnt, int rCnt) {
    if (lCnt > 0 || is90Right(sv,lCnt,rCnt)) return false;
    return (rCnt==1) && (sv[3] || sv[4]);
}
bool isUTurnLeft (int* sv, int lCnt, int rCnt) { return (rCnt==0)&&(lCnt>=2)&&sv[0]&&sv[1]; }
bool isUTurnRight(int* sv, int lCnt, int rCnt) { return (lCnt==0)&&(rCnt>=2)&&sv[4]&&sv[3]; }

// ================================================================
//  T-JUNCTION SHAPE DETECTOR
//  Returns: +1 = right branch present, -1 = left only, 0 = straight
// ================================================================
int detectTShape(int lCnt, int rCnt, int active) {
    if (rCnt >= 1) return  1;
    if (lCnt >= 1) return -1;
    return (active >= 4) ? 1 : 0;
}

// ================================================================
//  CENTER FOUND  (s1, s2, or s3)
// ================================================================
bool centerFound(int* sv) { return sv[1] || sv[2] || sv[3]; }

// ================================================================
//  ENTER / EXIT TURN
// ================================================================
void enterTurn(RobotState st, int side, unsigned long now) {
    robotState    = st;
    turnEnteredAt = now;
    lastSide      = side;
    clearHistory();
    resetAllConfirm();
    intgAcc = 0; filtD = 0; lastErr = 0;
}

void exitTurn(unsigned long now) {
    robotState   = ST_FOLLOW;
    turnExitedAt = now;
    resetAllConfirm();
    intgAcc = 0; filtD = 0; lastErr = 0;
    go(58, 58);
    delay(25);
}

// ================================================================
//  EXECUTE QUEUED JUNCTION ACTION
//  J_LEFT is the primary action; falls back to right if no
//  left branch is present.
// ================================================================
static int savedTShape = 0;

void executeJunctionAction(JunctionAction act, int tShape, unsigned long now) {
    if (act == J_STOP) { robotState = ST_STOP; stopMotors(); return; }

    if (act == J_LEFT) {
        // Left branch present (tShape <= 0) → turn left
        // No left branch (tShape == +1, right only) → fall back right
        if (tShape <= 0) { enterTurn(ST_TURN_L, -1, now); go(PIVOT_L_REV, PIVOT_L_FWD); }
        else             { enterTurn(ST_TURN_R,  1, now); go(PIVOT_R_FWD, PIVOT_R_REV); }
        return;
    }

    if (act == J_RIGHT) {
        if (tShape >= 0) { enterTurn(ST_TURN_R,  1, now); go(PIVOT_R_FWD, PIVOT_R_REV); }
        else             { enterTurn(ST_TURN_L, -1, now); go(PIVOT_L_REV, PIVOT_L_FWD); }
        return;
    }

    if (act == J_UTURN) {
        enterTurn(ST_UTURN, lastSide, now);
        if (lastSide < 0) go(UTURN_L_REV, UTURN_L_FWD);
        else              go(UTURN_L_FWD, UTURN_L_REV);
        return;
    }

    // J_STRAIGHT
    robotState = ST_JUNCTION; junctionEnteredAt = now; turnExitedAt = now;
    go(SPD_JUNCTION, SPD_JUNCTION);
}

// ================================================================
//  PID
//  v16: integral bleed zone widened — absErr < 220, decay 0.65
// ================================================================
void computePID(int error, int absErr, int* outL, int* outR) {
    int zone = (absErr > ZONE_SHARP) ? 2 : (absErr > ZONE_CURVE) ? 1 : 0;

    int rawD = constrain(error - lastErr, -1000, 1000);
    lastErr  = error;
    filtD    = ALPHA * filtD + (1.0f - ALPHA) * rawD;

    intgAcc = constrain(intgAcc + error, -INTG_LIMIT, INTG_LIMIT);
    // v16: wider bleed zone (220 vs 150) and faster decay (0.65 vs 0.75)
    if (absErr < 220) intgAcc *= 0.65f;

    float corr = KP[zone]*error + KI_GAIN*intgAcc + KD[zone]*filtD;
    corr = constrain(corr, -(float)CORR_LIMIT[zone], (float)CORR_LIMIT[zone]);
    if (error < -ZONE_CURVE)
        corr = constrain(corr*1.15f, -(float)CORR_LIMIT[zone], (float)CORR_LIMIT[zone]);

    int derivSlowdown = (int)(abs(filtD) * 0.50f);
    int errSlowdown   = absErr / 6;
    const int BASE_SPD[3] = { SPD_STRAIGHT, SPD_CURVE, SPD_SHARP };
    int spd = BASE_SPD[zone] - errSlowdown - derivSlowdown;
    spd     = constrain(spd, 0, SPD_STRAIGHT);
    if (abs(rawD) > 120) spd = max((int)(spd * 0.72f), 0);

    int prevE = eBuf[(hIdx - 2 + HIST) % HIST];
    if ((error > 0 && prevE < 0) || (error < 0 && prevE > 0)) corr *= 1.05f;

    *outL = spd + (int)corr;
    *outR = spd - (int)corr;
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    pinMode(rPwm, OUTPUT); pinMode(rF, OUTPUT); pinMode(rB, OUTPUT);
    pinMode(lPwm, OUTPUT); pinMode(lF, OUTPUT); pinMode(lB, OUTPUT);
    for (int i = 0; i < 5; i++) pinMode(sensorPins[i], INPUT);
    clearHistory();
    stopMotors();

    bool ready = false;
    while (!ready) {
        int c = 0;
        for (int i = 1; i <= 3; i++)
            if (digitalRead(sensorPins[i]) == LINE_DETECTED) c++;
        if (c >= 2) ready = true;
        delay(10);
    }
    delay(600);
}

// ================================================================
//  MAIN LOOP
// ================================================================
void loop() {

    // ── READ SENSORS ─────────────────────────────────────────────
    int sv[5];
    int wSum = 0, active = 0;
    for (int i = 0; i < 5; i++) {
        sv[i] = (digitalRead(sensorPins[i]) == LINE_DETECTED) ? 1 : 0;
        if (sv[i]) { wSum += W[i]; active++; }
    }
    int lCnt        = sv[0] + sv[1];
    int rCnt        = sv[3] + sv[4];
    uint8_t bitmask = buildMask(sv);
    unsigned long now = millis();

    // ── DEBUG every 200 ms ───────────────────────────────────────
    static unsigned long lastDbg = 0;
    if (now - lastDbg > 200) {
        lastDbg = now;
        Serial.printf("STATE=%d J=%d ACT=%d ERR=%d MASK=0x%02X L=%d R=%d\n",
            (int)robotState, junctionCount,
            active, (active ? wSum/active : 0), bitmask, lCnt, rCnt);
    }

    // ================================================================
    //  ACTIVE TURN STATES
    // ================================================================

    if (robotState == ST_TURN_L) {
        if ((now - turnEnteredAt) >= TURN90_MIN_MS && centerFound(sv)) {
            exitTurn(now); return;
        }
        go(PIVOT_L_REV, PIVOT_L_FWD);
        syncPID(active ? wSum/active : lastErr);
        return;
    }

    if (robotState == ST_TURN_R) {
        if ((now - turnEnteredAt) >= TURN90_MIN_MS && centerFound(sv)) {
            exitTurn(now); return;
        }
        go(PIVOT_R_FWD, PIVOT_R_REV);
        syncPID(active ? wSum/active : lastErr);
        return;
    }

    if (robotState == ST_ARC_L) {
        if ((now - turnEnteredAt) >= TURN45_MIN_MS && centerFound(sv)) {
            exitTurn(now); return;
        }
        go(ARC_L_IN, ARC_L_OUT);
        syncPID(active ? wSum/active : lastErr);
        return;
    }

    if (robotState == ST_ARC_R) {
        if ((now - turnEnteredAt) >= TURN45_MIN_MS && centerFound(sv)) {
            exitTurn(now); return;
        }
        go(ARC_R_OUT, ARC_R_IN);
        syncPID(active ? wSum/active : lastErr);
        return;
    }

    if (robotState == ST_UTURN) {
        if ((now - turnEnteredAt) >= UTURN_MIN_MS && centerFound(sv)) {
            exitTurn(now); return;
        }
        if (lastSide < 0) go(UTURN_L_REV, UTURN_L_FWD);
        else              go(UTURN_L_FWD, UTURN_L_REV);
        syncPID(active ? wSum/active : lastErr);
        return;
    }

    // ── J_STRAIGHT hold ──────────────────────────────────────────
    if (robotState == ST_JUNCTION) {
        if ((now - junctionEnteredAt) >= JUNCTION_HOLD_MS) {
            robotState = ST_FOLLOW; turnExitedAt = now; resetAllConfirm();
        } else { go(SPD_JUNCTION, SPD_JUNCTION); }
        return;
    }

    // ── Junction advance: drive to intersection centre then turn ─
    if (robotState == ST_JUNCTION_ADV) {
        if ((now - junctionAdvAt) >= JUNCTION_ADVANCE_MS) {
            Serial.printf(">>> ADV DONE action=%d tShape=%d\n",
                          (int)pendingAction, savedTShape);
            executeJunctionAction(pendingAction, savedTShape, now);
        } else { go(SPD_ADV, SPD_ADV); }
        return;
    }

    // ================================================================
    //  JUNCTION DETECTION  (4+ of 5 sensors lit)
    // ================================================================
    if (active >= JUNCTION_SENSORS) {
        linePresent = true;
        resetAllConfirm();
        intgAcc *= 0.2f;

        bool cooldownDone = (now - turnExitedAt)    > POST_TURN_COOLDOWN
                         && (now - lastJunctionTime) > JUNCTION_DEBOUNCE_MS;

        if (cooldownDone && junctionCount < JUNCTION_SEQ_LEN) {
            lastJunctionTime = now;
            JunctionAction act = JUNCTION_SEQ[junctionCount];
            Serial.printf(">>> JUNCTION #%d action=%d\n", junctionCount, (int)act);
            junctionCount++;

            if (act == J_STOP) { robotState = ST_STOP; stopMotors(); return; }

            savedTShape   = detectTShape(lCnt, rCnt, active);
            pendingAction = act;
            robotState    = ST_JUNCTION_ADV;
            junctionAdvAt = now;
            go(SPD_ADV, SPD_ADV);
            return;
        } else {
            go(SPD_JUNCTION, SPD_JUNCTION);
            return;
        }
    }

    // ================================================================
    //  LINE COMPLETELY LOST
    // ================================================================
    if (active == 0) {
        if (linePresent) {
            linePresent = false; intgAcc = 0;
            robotState  = ST_LOST; recHitCount = 0;
            resetAllConfirm();
        }
        go(-LOST_SPIN_SPD, LOST_SPIN_SPD);
        return;
    }

    // ── Line found again ─────────────────────────────────────────
    if (robotState == ST_LOST) {
        recHitCount++;
        if (recHitCount >= REC_HIT_NEEDED) {
            linePresent = true; robotState = ST_FOLLOW;
            recHitCount = 0; clearHistory();
        } else { go(-LOST_SPIN_SPD, LOST_SPIN_SPD); return; }
    } else {
        linePresent = true; recHitCount = 0;
    }

    int error  = wSum / active;
    int absErr = abs(error);
    pushErr(error);
    if (error < -100) lastSide = -1;
    if (error >  100) lastSide =  1;

    // ================================================================
    //  TURN DETECTION
    //
    //  STRUCTURE:
    //  ① U-turn  (single side — mutually exclusive, no priority needed)
    //  ② BOTH SIDES LIT → LEFT PRIORITY
    //       Fires ONLY when lCnt >= 1 AND rCnt >= 1 simultaneously.
    //       Commits LEFT turn after BOTH_SIDES_CONFIRM frames.
    //       Single-side turns are completely unaffected.
    //  ③ Single-side 90° right  (lCnt == 0, natural)
    //  ④ Single-side 90° left   (rCnt == 0, natural)
    //  ⑤ Single-side 45° right  (natural)
    //  ⑥ Single-side 45° left   (natural)
    // ================================================================
    bool inCooldown    = (now - turnExitedAt) < POST_TURN_COOLDOWN;
    bool turnCommitted = false;

    if (!inCooldown) {

        // ── ① U-TURN ─────────────────────────────────────────────
        if (isUTurnRight(sv, lCnt, rCnt) && absErr > 1500) {
            confirmUT++;
            confirm90L = confirm90R = confirm45L = confirm45R = confirmBothL = 0;
            if (confirmUT >= UTURN_CONFIRM) {
                enterTurn(ST_UTURN, 1, now);
                go(UTURN_L_FWD, UTURN_L_REV);
                turnCommitted = true;
            }
        } else if (isUTurnLeft(sv, lCnt, rCnt) && absErr > 1500) {
            confirmUT++;
            confirm90L = confirm90R = confirm45L = confirm45R = confirmBothL = 0;
            if (confirmUT >= UTURN_CONFIRM) {
                enterTurn(ST_UTURN, -1, now);
                go(UTURN_L_REV, UTURN_L_FWD);
                turnCommitted = true;
            }
        } else {
            confirmUT = 0;
        }

        // ── ② BOTH SIDES LIT — LEFT PRIORITY ─────────────────────
        //    Triggers ONLY when lCnt >= 1 AND rCnt >= 1.
        //    Single-side paths are not affected at all.
        if (!turnCommitted) {
            bool bothSides = (lCnt >= 1) && (rCnt >= 1);
            if (bothSides) {
                confirmBothL++;
                confirm90L = confirm90R = confirm45L = confirm45R = 0;
                if (confirmBothL >= BOTH_SIDES_CONFIRM) {
                    Serial.printf(">>> BOTH SIDES → LEFT PRIORITY lCnt=%d rCnt=%d\n",
                                  lCnt, rCnt);
                    enterTurn(ST_TURN_L, -1, now);       // LEFT turn
                    go(PIVOT_L_REV, PIVOT_L_FWD);
                    turnCommitted = true;
                }
            } else {
                confirmBothL = 0;
            }
        }

        // ── ③ 90° RIGHT  (single side, lCnt must be 0) ───────────
        if (!turnCommitted) {
            if (is90Right(sv, lCnt, rCnt)) {
                confirm90R++;
                confirm90L = confirm45L = confirm45R = 0;
                if (confirm90R >= TURN90_CONFIRM) {
                    enterTurn(ST_TURN_R, 1, now);
                    go(PIVOT_R_FWD, PIVOT_R_REV);
                    turnCommitted = true;
                }
            } else { confirm90R = 0; }
        }

        // ── ④ 90° LEFT  (single side, rCnt must be 0) ────────────
        if (!turnCommitted) {
            if (is90Left(sv, lCnt, rCnt)) {
                confirm90L++;
                confirm90R = confirm45L = confirm45R = 0;
                if (confirm90L >= TURN90_CONFIRM) {
                    enterTurn(ST_TURN_L, -1, now);
                    go(PIVOT_L_REV, PIVOT_L_FWD);
                    turnCommitted = true;
                }
            } else { confirm90L = 0; }
        }

        // ── ⑤ 45° RIGHT (single side) ────────────────────────────
        if (!turnCommitted) {
            if (is45Right(sv, lCnt, rCnt)) {
                confirm45R++;
                confirm45L = 0;
                if (confirm45R >= TURN45_CONFIRM) {
                    enterTurn(ST_ARC_R, 1, now);
                    go(ARC_R_OUT, ARC_R_IN);
                    turnCommitted = true;
                }
            } else { confirm45R = 0; }
        }

        // ── ⑥ 45° LEFT (single side) ─────────────────────────────
        if (!turnCommitted) {
            if (is45Left(sv, lCnt, rCnt)) {
                confirm45L++;
                confirm45R = 0;
                if (confirm45L >= TURN45_CONFIRM) {
                    enterTurn(ST_ARC_L, -1, now);
                    go(ARC_L_IN, ARC_L_OUT);
                    turnCommitted = true;
                }
            } else { confirm45L = 0; }
        }

    } // end !inCooldown

    if (turnCommitted) return;

    // ================================================================
    //  BITMASK PATTERN LOOKUP + PID
    // ================================================================
    int pidx = lookupPattern(bitmask);
    if (pidx >= 0) {
        bool edgeActive = (sv[0] || sv[4]);
        int lSpd = PATTERN_TABLE[pidx].lSpd;
        int rSpd = PATTERN_TABLE[pidx].rSpd;
        if (edgeActive && abs(lSpd) < MAX_PWW) {
            float f = (float)SPD_APPROACH / (float)SPD_STRAIGHT;
            lSpd = (int)(lSpd * f);
            rSpd = (int)(rSpd * f);
        }
        go(lSpd, rSpd);
        syncPID(error);
        return;
    }

    // ADAPTIVE PID
    int outL, outR;
    computePID(error, absErr, &outL, &outR);
    go(outL, outR);
}
