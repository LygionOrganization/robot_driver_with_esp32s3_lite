#include "Config.h"
#include "jointsCtrl.h"

void JointsCtrl::init(int baud) {
    baudrate = baud;
#ifndef TTL_ADAPTER_MACHINE
    Serial1.begin(baud, SERIAL_8N1, BUS_SERVO_RX, BUS_SERVO_TX);
    sc.pSerial = &Serial1;
    smst.pSerial = &Serial1;
    hl.pSerial = &Serial1;
#else
    Serial0.begin(500000);
    sc.pSerial = &Serial0;
    smst.pSerial = &Serial0;
    hl.pSerial = &Serial0;
#endif

#ifdef USE_HUB_MOTORS
    gqdmd.begin(&Serial1);
    gqdmd.setTxEnd_T32(1000000);
    gqdmd.setTimeOut(2);
#endif

    while(!Serial1) {}
}

void JointsCtrl::setBaudRate(int baud) {
    baudrate = baud;
    Serial1.updateBaudRate(baud);

}

bool JointsCtrl::setJointType(u_int8_t type) {
    switch (type) {
        case JOINT_TYPE_SC:
            jointType = type;
            return true;
            break;
        case JOINT_TYPE_SMST:
            jointType = type;
            return true;
            break;
        case JOINT_TYPE_HL:
            jointType = type;
            return true;
            break;
    } 
    return false;
}

bool JointsCtrl::setEncoderStepRange(u_int16_t steps, double angle) {
    if (steps < 512) {
        return false;
    }
    if (angle < 90 || angle > 360) {
        return false;
    }
    jointSteps = steps;
    middleSteps = round(jointSteps/2 - 1);
    jointRangeAngle = angle;
    jointRangeRad = jointRangeAngle * M_PI / 180.0;

    for(int i = 0; i < JOINTS_NUM; i++) {
        jointsZeroPos[i] = middleSteps; // or any other default value
        jointsFeedbackPos[i] = middleSteps; // or any other default value
        jointsCurrentPos[i] = middleSteps; // or any other default value
        jointsGoalPos[i] = middleSteps; // or any other default value
        jointsLastPos[i] = middleSteps; // or any other default value
    }

    return true;
}

int* JointsCtrl::singleFeedBack(u_int8_t id) {
    // Get feedback from servo
    // [0]ping status
    // [1]position
    // [2]speed
    // [3]load
    // [4]voltage
    // [5]temperature
    // [6]moving
    // [7]current

    static int feedback[SERVO_FEEDBACK_NUM];
    switch (jointType) {
        case JOINT_TYPE_SC:
            sc.FeedBack(id);
            if (sc.getLastError()) {
                feedback[FB_PING] = -1;
                break;
            }
            feedback[FB_PING] = 1;
            feedback[FB_POS] = sc.ReadPos(-1);
            feedback[FB_SPD] = sc.ReadSpeed(-1);
            feedback[FB_LOAD] = sc.ReadLoad(-1);
            feedback[FB_VOLT] = sc.ReadVoltage(-1);
            feedback[FB_TEMP] = sc.ReadTemper(-1);
            feedback[FB_MOVE] = sc.ReadMove(-1);
            feedback[FB_CURT] = sc.ReadCurrent(-1);
            break;
        case JOINT_TYPE_SMST:
            smst.FeedBack(id);
            if (smst.getLastError()) {
                feedback[FB_PING] = -1;
                break;
            }
            feedback[FB_PING] = 1;
            feedback[FB_POS] = smst.ReadPos(-1);
            feedback[FB_SPD] = smst.ReadSpeed(-1);
            feedback[FB_LOAD] = smst.ReadLoad(-1);
            feedback[FB_VOLT] = smst.ReadVoltage(-1);
            feedback[FB_TEMP] = smst.ReadTemper(-1);
            feedback[FB_MOVE] = smst.ReadMove(-1);
            feedback[FB_CURT] = smst.ReadCurrent(-1);
            break;
        case JOINT_TYPE_HL:
            hl.FeedBack(id);
            if (hl.getLastError()) {
                feedback[FB_PING] = -1;
                break;
            }
            feedback[FB_PING] = 1;
            feedback[FB_POS] = hl.ReadPos(-1);
            feedback[FB_SPD] = hl.ReadSpeed(-1);
            feedback[FB_LOAD] = hl.ReadLoad(-1);
            feedback[FB_VOLT] = hl.ReadVoltage(-1);
            feedback[FB_TEMP] = hl.ReadTemper(-1);
            feedback[FB_MOVE] = hl.ReadMove(-1);
            feedback[FB_CURT] = hl.ReadCurrent(-1);
            break;
    }

    return feedback;
}

int* JointsCtrl::feedbackSTSM(u_int8_t id) {
    // Get feedback from servo
    // [0]ping status
    // [1]position
    // [2]speed
    // [3]load
    // [4]voltage
    // [5]temperature
    // [6]moving
    // [7]current

    static int feedback[SERVO_FEEDBACK_NUM];
    smst.FeedBack(id);
    if (smst.getLastError()) {
        feedback[FB_PING] = -1;
        return feedback;
    }
    feedback[FB_PING] = 1;
    feedback[FB_POS] = smst.ReadPos(-1);
    feedback[FB_SPD] = smst.ReadSpeed(-1);
    feedback[FB_LOAD] = smst.ReadLoad(-1);
    feedback[FB_VOLT] = smst.ReadVoltage(-1);
    feedback[FB_TEMP] = smst.ReadTemper(-1);
    feedback[FB_MOVE] = smst.ReadMove(-1);
    feedback[FB_CURT] = smst.ReadCurrent(-1);

    return feedback;
}

int* JointsCtrl::feedbackHL(u_int8_t id) {
    // Get feedback from servo
    // [0]ping status
    // [1]position
    // [2]speed
    // [3]load
    // [4]voltage
    // [5]temperature
    // [6]moving
    // [7]current

    static int feedback[SERVO_FEEDBACK_NUM];
    hl.FeedBack(id);
    if (hl.getLastError()) {
        feedback[FB_PING] = -1;
        return feedback;
    }
    feedback[FB_PING] = 1;
    feedback[FB_POS] = hl.ReadPos(-1);
    feedback[FB_SPD] = hl.ReadSpeed(-1);
    feedback[FB_LOAD] = hl.ReadLoad(-1);
    feedback[FB_VOLT] = hl.ReadVoltage(-1);
    feedback[FB_TEMP] = hl.ReadTemper(-1);
    feedback[FB_MOVE] = hl.ReadMove(-1);
    feedback[FB_CURT] = hl.ReadCurrent(-1);

    return feedback;
}

int* JointsCtrl::feedbackSC(u_int8_t id) {
    // Get feedback from servo
    // [0]ping status
    // [1]position
    // [2]speed
    // [3]load
    // [4]voltage
    // [5]temperature
    // [6]moving
    // [7]current

    static int feedback[SERVO_FEEDBACK_NUM];
    sc.FeedBack(id);
    if (sc.getLastError()) {
        feedback[FB_PING] = -1;
        return feedback;
    }
    feedback[FB_PING] = 1;
    feedback[FB_POS] = sc.ReadPos(-1);
    feedback[FB_SPD] = sc.ReadSpeed(-1);
    feedback[FB_LOAD] = sc.ReadLoad(-1);
    feedback[FB_VOLT] = sc.ReadVoltage(-1);
    feedback[FB_TEMP] = sc.ReadTemper(-1);
    feedback[FB_MOVE] = sc.ReadMove(-1);
    feedback[FB_CURT] = sc.ReadCurrent(-1);

    return feedback;
}


bool JointsCtrl::pingSTSM(u_int8_t id) {
    smst.Ping(id);
    if (smst.getLastError()) {
        return false;
    } else {
        return true;
    }
}

bool JointsCtrl::pingHL(u_int8_t id) {
    hl.Ping(id);
    if (hl.getLastError()) {
        return false;
    } else {
        return true;
    }
}

bool JointsCtrl::pingSC(u_int8_t id) {
    sc.Ping(id);
    if (sc.getLastError()) {
        return false;
    } else {
        return true;
    }
}

bool JointsCtrl::ping(u_int8_t id) {
    switch (jointType) {
        case JOINT_TYPE_SC:
            pingSC(id);
            break;
        case JOINT_TYPE_SMST:
            pingSTSM(id);
            break;
        case JOINT_TYPE_HL:
            pingHL(id);
            break;
    }
    return false;
}


bool JointsCtrl::changeIDSTSM(u_int8_t old_id, u_int8_t new_id) {
    if (!pingSTSM(old_id)) {
        return false;
    } else {
        smst.unLockEprom(old_id);
        smst.writeByte(old_id, SMS_STS_ID, new_id);
        smst.LockEprom(new_id);
        return true;
    }
}

bool JointsCtrl::changeIDHL(u_int8_t old_id, u_int8_t new_id) {
    if (!pingHL(old_id)) {
        return false;
    } else {
        hl.unLockEprom(old_id);
        hl.writeByte(old_id, SMS_STS_ID, new_id); // change address
        hl.LockEprom(new_id);
        return true;
    }
}

bool JointsCtrl::changeIDSC(u_int8_t old_id, u_int8_t new_id) {
    if (!pingSC(old_id)) {
        return false;
    } else {
        sc.unLockEprom(old_id);
        sc.writeByte(old_id, SCSCL_ID, new_id);  // change address
        sc.LockEprom(new_id);
        return true;
    }
}

bool JointsCtrl::changeID(u_int8_t old_id, u_int8_t new_id) {
    switch (jointType) {
        case JOINT_TYPE_SC:
            if (!pingSC(old_id)) {
                return false;
            } else {
                changeIDSC(old_id, new_id);
                return true;
            }
            break;
        case JOINT_TYPE_SMST:
            if (!pingSTSM(old_id)) {
                return false;
            } else {
                changeIDSTSM(old_id, new_id);
                return true;
            }
            break;
        case JOINT_TYPE_HL:
            if (!pingHL(old_id)) {
                return false;
            } else {
                changeIDHL(old_id, new_id);
                return true;
            }
            break;
    }
    return false;
}


bool JointsCtrl::setMiddleSTSM(u_int8_t id) {
    if (!pingSTSM(id)) {
        return false;
    } else {
        smst.CalibrationOfs(id);
        return true;
    }
}

bool JointsCtrl::setMiddleHL(u_int8_t id) {
    if (!ping(id)) {
        return false;
    } else {
        hl.CalibrationOfs(id);
        return true;
    }
}

bool JointsCtrl::setMiddle(u_int8_t id) {
    switch (jointType) {
        case JOINT_TYPE_SC:
            return false;
            break;
        case JOINT_TYPE_SMST:
            return setMiddleSTSM(id);
            break;
        case JOINT_TYPE_HL:
            return setMiddleHL(id);
            break;
    }
    return false;
}


void JointsCtrl::moveMiddle(u_int8_t id) {
    switch (jointType) {
        case JOINT_TYPE_SC:
            sc.WritePos(id, middleSteps, 0, 300);
            break;
        case JOINT_TYPE_SMST:
            smst.WritePosEx(id, middleSteps, 300, 50);
            break;
        case JOINT_TYPE_HL:
            hl.WritePosEx(id, middleSteps, 10, 50, 500);
            break;
    }
}


void JointsCtrl::torqueLockSTSM(u_int8_t id, bool state) {
    if (state) {
        smst.EnableTorque(id, 1);
    } else {
        smst.EnableTorque(id, 0);
    }
}

void JointsCtrl::torqueLockHL(u_int8_t id, bool state) {
    if (state) {
        hl.EnableTorque(id, 1);
    } else {
        hl.EnableTorque(id, 0);
    }
}

void JointsCtrl::torqueLockSC(u_int8_t id, bool state) {
    if (state) {
        sc.EnableTorque(id, 1);
    } else {
        sc.EnableTorque(id, 0);
    }
}

void JointsCtrl::torqueLock(u_int8_t id, bool state) {
    switch (jointType) {
        case JOINT_TYPE_SC:
            torqueLockSC(id, state);
            break;
        case JOINT_TYPE_SMST:
            torqueLockSTSM(id, state);
            break;
        case JOINT_TYPE_HL:
            torqueLockHL(id, state);
            break;
    }
}

void JointsCtrl::stepsCtrlSC(u_int8_t id, int pos, int time, int speed, bool move_trigger) {
    if (move_trigger) {
        sc.WritePos(id, pos, time, speed);
    } else {
        sc.RegWritePos(id, pos, time, speed);
    }
}

void JointsCtrl::stepsCtrlSMST(u_int8_t id, int pos, int speed, int acc, bool move_trigger) {
    if (move_trigger) {
        smst.WritePosEx(id, pos, speed, acc);
    } else {
        smst.RegWritePosEx(id, pos, speed, acc);
    }
}

void JointsCtrl::stepsCtrlHL(u_int8_t id, int pos, int speed, int acc, int currt_limit, bool move_trigger) {
    if (move_trigger) {
        hl.WritePosEx(id, pos, speed, acc, currt_limit);
    } else {
        hl.RegWritePosEx(id, pos, speed, acc, currt_limit);
    }
}

double JointsCtrl::mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int JointsCtrl::angleCtrlSC(u_int8_t id, int mid_pos, double angle, double speed, bool move_trigger) {
    int goal_pos = mid_pos + round(mapDouble(angle, 0, jointRangeAngle, 0, jointSteps));
    int goal_spd = round(mapDouble(speed, 0, jointRangeAngle, 0, jointSteps));
    if (move_trigger) {
        sc.WritePos(id, goal_pos, 0, goal_spd);
    } else {
        sc.RegWritePos(id, goal_pos, 0, goal_spd);
    }
    return goal_pos;
}

int JointsCtrl::angleCtrlSMST(u_int8_t id, double angle, double speed, double acc, bool move_trigger) {
    int goal_pos = middleSteps + round(mapDouble(angle, 0, jointRangeAngle, 0, jointSteps));
    int goal_spd = round(mapDouble(speed, 0, jointRangeAngle, 0, jointSteps));
    int goal_acc = round(mapDouble(acc, 0, jointRangeAngle, 0, jointSteps));
    if (move_trigger) {
        smst.WritePosEx(id, goal_pos, goal_spd, goal_acc);
    } else {
        smst.RegWritePosEx(id, goal_pos, goal_spd, goal_acc);
    }
    return goal_pos;
}

int JointsCtrl::angleCtrlHL(u_int8_t id, double angle, double speed, double acc, int currt_limit, bool move_trigger) {
    int goal_pos = middleSteps + round(mapDouble(angle, 0, jointRangeAngle, 0, jointSteps));
    int goal_spd = round(mapDouble(speed, 0, jointRangeAngle, 0, jointSteps));
    int goal_acc = round(mapDouble(acc, 0, jointRangeAngle, 0, jointSteps));
    if (move_trigger) {
        hl.WritePosEx(id, goal_pos, goal_spd, goal_acc, currt_limit);
    } else {
        hl.RegWritePosEx(id, goal_pos, goal_spd, goal_acc, currt_limit);
    }
    return goal_pos;
}

int JointsCtrl::radCtrlSC(u_int8_t id, int mid_pos, double rad, double speed, bool move_trigger) {
    int goal_pos = mid_pos + round(mapDouble(rad, 0, jointRangeRad, 0, jointSteps));
    int goal_spd = round(mapDouble(speed, 0, jointRangeRad, 0, jointSteps));
    if (move_trigger) {
        sc.WritePos(id, goal_pos, 0, goal_spd);
    } else {
        sc.RegWritePos(id, goal_pos, 0, goal_spd);
    }
    return goal_pos;
}

int JointsCtrl::radCtrlSMST(u_int8_t id, double rad, double speed, double acc, bool move_trigger) {
    int goal_pos = middleSteps + round(mapDouble(rad, 0, jointRangeRad, 0, jointSteps));
    int goal_spd = round(mapDouble(speed, 0, jointRangeRad, 0, jointSteps));
    int goal_acc = round(mapDouble(acc, 0, jointRangeRad, 0, jointSteps));
    if (move_trigger) {
        smst.WritePosEx(id, goal_pos, goal_spd, goal_acc);
    } else {
        smst.RegWritePosEx(id, goal_pos, goal_spd, goal_acc);
    }
    return goal_pos;
}

int JointsCtrl::radCtrlHL(u_int8_t id, double rad, double speed, double acc, int currt_limit, bool move_trigger) {
    int goal_pos = middleSteps + round(mapDouble(rad, 0, jointRangeRad, 0, jointSteps));
    int goal_spd = round(mapDouble(speed, 0, jointRangeRad, 0, jointSteps));
    int goal_acc = round(mapDouble(acc, 0, jointRangeRad, 0, jointSteps));
    if (move_trigger) {
        hl.WritePosEx(id, goal_pos, goal_spd, goal_acc, currt_limit);
    } else {
        hl.RegWritePosEx(id, goal_pos, goal_spd, goal_acc, currt_limit);
    }
    return goal_pos;
}

void JointsCtrl::moveTrigger() {
    switch (jointType) {
        case JOINT_TYPE_SC:
            sc.RegWriteAction();
            break;
        case JOINT_TYPE_SMST:
            smst.RegWriteAction();
            break;
        case JOINT_TYPE_HL:
            hl.RegWriteAction();
            break;
    }
}

// hub motor ctrl
void JointsCtrl::hubMotorCtrl(int spd_1, int spd_2, int spd_3, int spd_4) {
#ifdef USE_HUB_MOTORS
    gqdmd.SpeedCtl(1, spd_1, 500, 600, 200);
    gqdmd.SpeedCtl(2, spd_2, 500, 600, 200);
    gqdmd.SpeedCtl(3, spd_3, 500, 600, 200);
    gqdmd.SpeedCtl(4, spd_4, 500, 600, 200);
#endif
}

// for applications: LyLinkArm
int* JointsCtrl::getJointsZeroPosArray() {
    return jointsZeroPos;
}

void JointsCtrl::setJointsZeroPosArray(int values[]) {
    for (int i = 0; i < JOINTS_NUM; i++) {
        jointsZeroPos[i] = values[i];
    }
}

int* JointsCtrl::getLinkArmPosSC() {
    for (int i = 0; i < JOINTS_NUM; i++) {
        jointsFeedbackPos[i] = sc.ReadPos(jointID[i]);
    }
    return jointsFeedbackPos;
}

int* JointsCtrl::getLinkArmTorqueSC() {
    for (int i = 0; i < JOINTS_NUM; i++) {
        int load = sc.ReadLoad(jointID[i]);
        jointsFeedbackTorque[i] = (abs(load) < 1e-3) ? 0 : load;
    }
    return jointsFeedbackTorque;
}

void JointsCtrl::setCurrentSCPosMiddle() {
    for (int i = 0; i < JOINTS_NUM; i++) {
        jointsFeedbackPos[i] = sc.ReadPos(jointID[i]);
        jointsZeroPos[i] = jointsFeedbackPos[i];
    }
}

// for applications: LyLinkArm
// joint_1 : as the base joint
// joint_2 : as the shoulder joint (base front)
// joint_3 : as the elbow joint (base back)
// joint_4 : as the gripper joint
// >-------O----O
// |       |   /
// 3       |  /
//       1-S E-2
//          B-0
void JointsCtrl::linkArmSCJointsCtrlAngle(double angles[]) {
    for (int i = 0; i < JOINTS_NUM; i++) {
        jointsCurrentPos[i] = angleCtrlSC(jointID[i], jointsZeroPos[i], angles[i], 0, false);
    }
    moveTrigger();
}

void JointsCtrl::linkArmSCJointsCtrlRad(double rads[]) {
    for (int i = 0; i < JOINTS_NUM; i++) {
        jointsCurrentPos[i] = radCtrlSC(jointID[i], jointsZeroPos[i], rads[i], 0, false);
    }
    moveTrigger();
}

bool JointsCtrl::linkArmPlaneIK(double x, double z) {
    // Calculate the angles for the joints based on the x and z coordinates
    double l_af = sqrt(pow(x, 2) + pow(z, 2));
    double theta = acos(- (pow(l_ab, 2) - pow(l_af, 2) - pow(l_bf, 2))/(2 * l_af * l_bf));
    double lambda = atan2(z, x);
    // output: the angle of the shoulder-front joint
    double alpha = 1.570796326794897 - theta - lambda;

    double omega = acos(-(pow(l_bf, 2) - pow(l_af, 2) - pow(l_ab, 2))/(2 * l_af * l_ab));
    double delta = atan2(x, z);
    // output: the radius of the eoat pitch
    double mu = delta + omega - 1.570796326794897;
    double l_ch = sin(mu) * l_ac;
    double l_ci = l_ch + z;
    double l_ah = cos(mu) * l_ac;
    double l_ei = x + l_ef - l_ah;
    double l_ce = sqrt(pow(l_ei, 2) + pow(l_ci, 2));
    double psi = acos(-(pow(l_cd, 2) - pow(l_ce, 2) - pow(l_de, 2))/(2 * l_ce * l_de));
    double epsilon = atan2(l_ci, l_ei);
    // output: the angle of the shoulder-rear joint
    double beta = epsilon + psi - 1.570796326794897;

    if (isnan(alpha) || isnan(beta) || isnan(theta)) {
        ik_status = false;
        return false;
    }
    ik_status = true;

    double armIKRad_1 = alpha - l_bf_rad;
    double armIKRad_2 = 1.570796326794897 - beta;

    if (armIKRad_1 >= jointMaxRads[1] || armIKRad_1 <= jointMinRads[1] ||
        armIKRad_2 >= jointMaxRads[2] || armIKRad_2 <= jointMinRads[2] ||
        mu >= jointMaxRads[3] || mu <= jointMinRads[3]) {
        ik_status = false;
        return false;
    }

    armIKRad[1] = armIKRad_1;
    armIKRad[2] = armIKRad_2;
    armIKRad[3] = mu;
    return true;
}

// use the xyzgIK to calculate the rbzgIK.
void JointsCtrl::spaceIK2FPVIK() {
    rbzgIK[0] = xyzgIK[0];
    rbzgIK[4] = xyzgIK[4];
    rbzgIK[2] = armIKRad[0];
    rbzgIK[1] = sqrt(pow(xyzgIK[1], 2) + pow(xyzgIK[2], 2)) - (l_ef / 2);
    rbzgIK[3] = xyzgIK[3];
}

// use the rbzgIK to calculate the xyzgIK.
void JointsCtrl::FPVIK2spaceIK() {
    xyzgIK[1] = rbzgIK[1] * cos(rbzgIK[2]) - (l_ef / 2);
    xyzgIK[2] = - rbzgIK[1] * sin(rbzgIK[2]);
    xyzgIK[3] = rbzgIK[3];
    xyzgIK[0] = rbzgIK[0];
    xyzgIK[4] = rbzgIK[4];
}

double* JointsCtrl::linkArmSpaceIK(double x, double y, double z, double g) {
    double armIKRad_0 = atan2(-y, x);
    if (armIKRad_0 >= jointMaxRads[1] || armIKRad_0 <= jointMinRads[1]) {
        xyzgIK[0] = -1;
        spaceIK2FPVIK();
        ik_status = false;
        return xyzgIK;
    }
    armIKRad[0] = armIKRad_0;
    linkArmPlaneIK(sqrt(pow(x, 2) + pow(y, 2)) - (l_ef / 2), z);
    if (ik_status) {
        radCtrlSC(jointID[0], jointsZeroPos[0], armIKRad[0], jointsMaxSpeed, false);
        radCtrlSC(jointID[1], jointsZeroPos[1], armIKRad[1], jointsMaxSpeed, false);
        radCtrlSC(jointID[2], jointsZeroPos[2], armIKRad[2], jointsMaxSpeed, false);
        angleCtrlSC(jointID[3], jointsZeroPos[3], g, 0, false);
        moveTrigger();
        xyzgIK[0] = 1;
        xyzgIK[1] = x;
        xyzgIK[2] = y;
        xyzgIK[3] = z;
        xyzgIK[4] = g;
        spaceIK2FPVIK();
        return xyzgIK;
    } else {
        xyzgIK[0] = -1;
        spaceIK2FPVIK();
        return xyzgIK;
    }
}

double* JointsCtrl::linkArmFPVIK(double r, double b, double z, double g) {
    if (b >= jointMaxRads[1] || b <= jointMinRads[1]) {
        rbzgIK[0] = -1;
        FPVIK2spaceIK();
        ik_status = false;
        return rbzgIK;
    }
    armIKRad[0] = b;
    linkArmPlaneIK(r - (l_ef / 2), z);
    if (ik_status) {
        radCtrlSC(jointID[0], jointsZeroPos[0], armIKRad[0], jointsMaxSpeed, false);
        radCtrlSC(jointID[1], jointsZeroPos[1], armIKRad[1], jointsMaxSpeed, false);
        radCtrlSC(jointID[2], jointsZeroPos[2], armIKRad[2], jointsMaxSpeed, false);
        angleCtrlSC(jointID[3], jointsZeroPos[3], g, 0, false);
        moveTrigger();
        rbzgIK[0] = 1;
        rbzgIK[1] = r;
        rbzgIK[2] = b;
        rbzgIK[3] = z;
        rbzgIK[4] = g;
        FPVIK2spaceIK();
        return rbzgIK;
    } else {
        rbzgIK[0] = -1;
        FPVIK2spaceIK();
        return rbzgIK;
    }
}

// ctrl the movement in a smooth way.
// |                 ..  <-end
// |             .    |
// |           .    
// |         .        |
// |        .
// |      .           |
// |. . <-start
// ----------------------
// 0                  1 rate
double JointsCtrl::smoothCtrl(double start, double end, double rate) {
    double out;
    out = (end - start)*((cos(rate*M_PI+M_PI)+1)/2) + start;
    return out;
}

double JointsCtrl::getSmoothStepsXYZ(double x, double y, double z) {
    double deltaPos[3] = {abs(x - xyzgIK_last[1]),
                          abs(y - xyzgIK_last[2]),
                          abs(z - xyzgIK_last[3])};
    double maxVal = deltaPos[0];
    for(int i = 0; i < (sizeof(deltaPos) / sizeof(deltaPos[0])); i++){
      maxVal = max(deltaPos[i],maxVal);
    }
    return maxVal;
}

double* JointsCtrl::smoothXYZGCtrl(double x, double y, double z, double g, double spd) {
    memcpy(xyzgIK_last, xyzgIK, sizeof(xyzgIK));
    for (double i=0; i<=1; i+=(spd/getSmoothStepsXYZ(x, y, z))) {
        linkArmSpaceIK(smoothCtrl(xyzgIK_last[1], x, i), 
                       smoothCtrl(xyzgIK_last[2], y, i), 
                       smoothCtrl(xyzgIK_last[3], z, i), 
                       g);
        if (xyzgIK[0] == -1) {
            memcpy(xyzgIK_last, xyzgIK, sizeof(xyzgIK));
            return xyzgIK;
        }
    }
    linkArmSpaceIK(x, y, z, g);
    memcpy(xyzgIK_last, xyzgIK, sizeof(xyzgIK));
    return xyzgIK;
}

double JointsCtrl::getSmoothStepsFPV(double r, double b, double z, double baseRate) {
    double deltaPos[3] = {abs(r - rbzgIK_last[1]),
                          abs(b - rbzgIK_last[2]) * baseRate,
                          abs(z - rbzgIK_last[3])};
    double maxVal = deltaPos[0];
    for(int i = 0; i < (sizeof(deltaPos) / sizeof(deltaPos[0])); i++){
      maxVal = max(deltaPos[i],maxVal);
    }
    return maxVal;
}

double* JointsCtrl::smoothFPVAbsCtrl(double r, double b, double z, double g, double spd, double baseRate) {
    memcpy(rbzgIK_last, rbzgIK, sizeof(rbzgIK));
    for (double i=0; i<=1; i+=(spd/getSmoothStepsFPV(r, b, z, baseRate))) {
        linkArmFPVIK(smoothCtrl(rbzgIK_last[1], r, i), 
                     smoothCtrl(rbzgIK_last[2], b, i), 
                     smoothCtrl(rbzgIK_last[3], z, i), 
                     g);
        if (rbzgIK[0] == -1) {
            memcpy(rbzgIK_last, rbzgIK, sizeof(rbzgIK));
            return rbzgIK;
        }
    }
    linkArmFPVIK(r, b, z, g);
    memcpy(rbzgIK_last, rbzgIK, sizeof(rbzgIK));
    return rbzgIK;
}

void JointsCtrl::setMaxJointsSpeed(double speed) {
    if (speed >= 0) {
        jointsMaxSpeed = speed;
    }
}

double* JointsCtrl::getJointFBRads() {
    static double jointRads[JOINTS_NUM];
    getLinkArmPosSC();
    for (int i = 0; i < JOINTS_NUM; i++) {
        jointRads[i] = mapDouble(jointsFeedbackPos[i] - jointsZeroPos[i], 0, jointSteps, 0, jointRangeRad) * jointDirection[i];
    }
    return jointRads;
}

double* JointsCtrl::getJointGoalRads() {
    return armIKRad;
}

double* JointsCtrl::getXYZGIK() {
    return xyzgIK;
}

double* JointsCtrl::getRBZGIK() {
    return rbzgIK;
}

void JointsCtrl::setLinkArmFeedbackFlag(bool flag, int hz) {
    linkArmFeedbackFlag = flag;
    linkArmFeedbackHz = hz;
}

bool JointsCtrl::linkArmPlaneFK(double alpha, double beta, double& x, double& z) {
    // Calculate the forward kinematics based on the joint angles alpha and beta
    double theta = 1.570796326794897 - alpha; // Recover theta from alpha
    double lambda = 1.570796326794897 - beta; // Recover lambda from beta

    // Calculate the length of l_af using the angles
    double l_af = sqrt(pow(l_ab, 2) + pow(l_bf, 2) - 2 * l_ab * l_bf * cos(theta));

    // Calculate the angle delta
    double delta = acos((pow(l_af, 2) + pow(l_ab, 2) - pow(l_bf, 2)) / (2 * l_af * l_ab));

    // Calculate the x and z coordinates
    x = l_af * cos(lambda + delta);
    z = l_af * sin(lambda + delta);

    // Check for invalid results
    if (isnan(x) || isnan(z)) {
        ik_status = false;
        return false;
    }

    ik_status = true;
    return true;
}

void JointsCtrl::ttlTestMachine() {
    while (true)
    {
        stepsCtrlSC(254, 200, 0, 0, 1);
        delay(1000);
        stepsCtrlSC(254, 900, 0, 0, 1);
        delay(1000);
    }
    
}

// TTL Node
int JointsCtrl::writeByte(u_int8_t id, u_int8_t addr, u8 data) {
    return smst.writeByte(id, addr, data);
}

void JointsCtrl::singleLedCtrl(u_int8_t id, u_int8_t num, u_int8_t r, u_int8_t g, u_int8_t b) {
    uint8_t r2 = (r * 4) / 256;
    uint8_t g3 = (g * 8) / 256;
    uint8_t b2 = (b * 4) / 256;

    uint8_t rgb232 = ((g3 << 5) | (r2 << 2) | b2) << 1;

    if (num == 1) {
        smst.writeByte(id, 43, rgb232);
    } else if (num == 2)
    {
        smst.writeByte(id, 44, rgb232);
    }
    smst.writeByte(id, 42, 2);
}

void JointsCtrl::allLedCtrl(u_int8_t id, u_int8_t r, u_int8_t g, u_int8_t b) {
    uint8_t r2 = (r * 4) / 256;
    uint8_t g3 = (g * 8) / 256;
    uint8_t b2 = (b * 4) / 256;

    uint8_t rgb232 = ((g3 << 5) | (r2 << 2) | b2) << 1;

    smst.writeByte(id, 43, rgb232);
    smst.writeByte(id, 44, rgb232);
    smst.writeByte(id, 42, 2);
}