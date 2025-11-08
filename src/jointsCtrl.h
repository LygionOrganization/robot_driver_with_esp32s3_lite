#ifndef JOINTS_CTRL_H
#define JOINTS_CTRL_H

#include <SCServo.h>
#ifdef USE_HUB_MOTORS
#include <GJWMotor.h>
#endif
#include <math.h>

// class SMS_STS_Public : public SMS_STS {
// public:
//     using SMS_STS::writeByte;
// };

#define JOINT_TYPE_SC 0
#define JOINT_TYPE_SMST 1
#define JOINT_TYPE_HL 2

#define JOINTS_NUM 4 // Number of joints
#define JOINTS_SC_MAX_POS 1023 // Maximum position for each joint in steps
#define JOINTS_SMST_MAX_POS 4095 // Maximum position for each joint in steps
#define JOINTS_HL_MAX_POS 4095 // Maximum position for each joint in steps
#define JOINTS_SC_RANGE_ANGLE 360.0 // Maximum angle for each joint in degrees
#define JOINTS_SMST_RANGE_ANGLE 360.0 // Maximum angle for each joint in degrees
#define JOINTS_HL_RANGE_ANGLE 360.0 // Maximum angle for each joint in degrees

#define SERVO_FEEDBACK_NUM 8 // Number of feedback parameters
#define FB_PING 0
#define FB_POS  1
#define FB_SPD  2
#define FB_LOAD 3
#define FB_VOLT 4
#define FB_TEMP 5
#define FB_MOVE 6
#define FB_CURT 7

// for applications: LyLinkArm
#define SERVO_ARRAY_0 31
#define SERVO_ARRAY_1 32
#define SERVO_ARRAY_2 33
#define SERVO_ARRAY_3 34
#define TTL_NODE_ID   40

// LyLinkArm LT
#define LINK_AB 224.0
#define LINK_BC 145.0
#define LINK_CD_1 24.0
#define LINK_CD_2 120.0
#define LINK_DE 120.0
#define LINK_EF 25.0
#define LINK_BF_1 24.0
#define LINK_BF_2 120.0


class JointsCtrl {
    private:
        SCSCL sc;
        SMS_STS smst;
        HLSCL hl;
#ifdef USE_HUB_MOTORS
        GQDMD gqdmd;
#endif
        // for applications: LyLinkArm

        // [0] base rad
        // [1] shoulder-front rad
        // [2] shoulder-rear rad
        // [3] eoat-pitch rad
        double armIKRad[JOINTS_NUM]; // array to store the IK radian of each joint
        double xyzgIK[JOINTS_NUM + 1]; // array to store the IK pos of each joint
        double rbzgIK[JOINTS_NUM + 1]; // array to store the IK pos of FPV Ctrl
        // smooth ctrl
        double xyzgIK_last[JOINTS_NUM + 1]; // array to store the last IK pos of each joint
        double rbzgIK_last[JOINTS_NUM + 1]; // array to store the last IK pos of FPV Ctrl

        double l_ab = LINK_AB; // length of link AB
        double l_bc = LINK_BC; // length of link BC
        double l_ac = l_ab + l_bc; // length of link AC
        double l_cd = sqrt(pow(LINK_CD_1, 2) + pow(LINK_CD_2, 2)); // length of link CD
        double l_de = LINK_DE; // length of link DE
        double l_ef = LINK_EF; // length of link EF
        double l_bf = sqrt(pow(LINK_BF_1, 2) + pow(LINK_BF_2, 2)); // length of link BF_1
        double l_bf_rad = atan2(LINK_BF_1, LINK_BF_2); // rad offset of link BF_1, ik output - offset = servo input

        bool ik_status = false; // ik status

                
        // the max speed of the joints(in rad/s)
        double jointsMaxSpeed = 0;
        
        // [0]ping status
        // [1]position
        // [2]speed
        // [3]load
        // [4]voltage
        // [5]temperature
        // [6]moving
        // [7]current
        int servoFeedback[SERVO_FEEDBACK_NUM]; // array to store the feedback from each servo

        u_int8_t jointType = JOINT_TYPE_SMST; // JOINT_TYPE_SC, JOINT_TYPE_SMST, JOINT_TYPE_HL
        u_int16_t jointSteps = JOINTS_SMST_MAX_POS; // steps in one circle
        u_int16_t middleSteps = jointSteps/2 - 1;
        double jointRangeAngle = JOINTS_SMST_RANGE_ANGLE;
        double jointRangeRad = jointRangeAngle * M_PI / 180.0; // wiggle range in radian

        double jointMaxRads[JOINTS_NUM] = {M_PI/2, M_PI/2, M_PI/2, M_PI/2}; // max rad of each joint
        double jointMinRads[JOINTS_NUM] = {-M_PI/2, -M_PI/2, -M_PI/4, -M_PI/2}; // min rad of each joint


    public:
        // for applications: LyLinkArm
        int jointsZeroPos[JOINTS_NUM]; // array to store the zero position of each joint
        int jointsFeedbackPos[JOINTS_NUM]; // array to store the feedback position from each joint
        int jointsFeedbackTorque[JOINTS_NUM]; // array to store the feedback torque from each joint
        int jointsCurrentPos[JOINTS_NUM]; // array to store the current position of each joint
        int jointsGoalPos[JOINTS_NUM]; // array to store the goal position of each joint
        int jointsLastPos[JOINTS_NUM]; // array to store the last position of each joint
        int jointID[JOINTS_NUM] = {SERVO_ARRAY_0, SERVO_ARRAY_1, SERVO_ARRAY_2, SERVO_ARRAY_3};
        int8_t jointDirection[JOINTS_NUM] = {1, 1, 1, 1}; // direction of each joint

        // for TTL Node
        int sbus[17];
        int writeByte(u_int8_t id, u_int8_t addr, u8 data);
        void singleLedCtrl(u_int8_t id, u_int8_t num, u_int8_t r, u_int8_t g, u_int8_t b);
        void allLedCtrl(u_int8_t id, u_int8_t r, u_int8_t g, u_int8_t b);
        int readSBUS();

        // for applications: LyLinkArm
        JointsCtrl() {
            // Initialize jointsZeroPos array
            for(int i = 0; i < JOINTS_NUM; i++) {
                jointsZeroPos[i] = middleSteps; // or any other default value
                jointsFeedbackPos[i] = middleSteps; // or any other default value
                jointsCurrentPos[i] = middleSteps; // or any other default value
                jointsGoalPos[i] = middleSteps; // or any other default value
                jointsLastPos[i] = middleSteps; // or any other default value
            }
        }

#ifdef DEV_TYPE_LINKARM_LT
        bool linkArmFeedbackFlag = true; // link arm feedback flag
#else
        bool linkArmFeedbackFlag = false;
#endif
        int linkArmFeedbackHz = 20; // link arm feedback Hz

        unsigned int baudrate = 1000000;
        bool fineTuningMode = false;
        bool torqueLockMode = true;
        bool espnowLeader = false;

        double fpv_r = 260.5;
        double fpv_b = 0;
        double fpv_z = 122.38;
        double fpv_g = 30;

        void init(int baud);
        void setBaudRate(int baud);
        bool setJointType(u_int8_t type);
        bool setEncoderStepRange(u_int16_t steps, double angle);

        int* feedbackSTSM(u_int8_t id);
        int* feedbackHL(u_int8_t id);
        int* feedbackSC(u_int8_t id);
        int* singleFeedBack(u_int8_t id);

        bool pingSTSM(u_int8_t id);
        bool pingHL(u_int8_t id);
        bool pingSC(u_int8_t id);
        bool ping(u_int8_t id);

        bool changeIDSTSM(u_int8_t old_id, u_int8_t new_id);
        bool changeIDHL(u_int8_t old_id, u_int8_t new_id);
        bool changeIDSC(u_int8_t old_id, u_int8_t new_id);
        bool changeID(u_int8_t old_id, u_int8_t new_id);

        bool setMiddleSTSM(u_int8_t id);
        bool setMiddleHL(u_int8_t id);
        bool setMiddleSC(u_int8_t id);
        bool setMiddle(u_int8_t id); // sc servo can not setMiddle pos

        void moveMiddle(u_int8_t id);

        void torqueLockSTSM(u_int8_t id, bool state);
        void torqueLockHL(u_int8_t id, bool state);
        void torqueLockSC(u_int8_t id, bool state);
        void torqueLock(u_int8_t id, bool state);

        void stepsCtrlSC(u_int8_t id, int pos, int time, int speed, bool move_trigger = true);
        void stepsCtrlSMST(u_int8_t id, int pos, int speed, int acc, bool move_trigger = true);
        void stepsCtrlHL(u_int8_t id, int pos, int speed, int acc, int currt_limit, bool move_trigger = true);
        
        double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);
        int angleCtrlSC(u_int8_t id, int mid_pos, double angle, double speed, bool move_trigger = true);
        int angleCtrlSMST(u_int8_t id, double angle, double speed, double acc, bool move_trigger = true);
        int angleCtrlHL(u_int8_t id, double angle, double speed, double acc, int currt_limit, bool move_trigger = true);
        int radCtrlSC(u_int8_t id, int mid_pos, double rad, double speed, bool move_trigger = true);
        int radCtrlSMST(u_int8_t id, double rad, double speed, double acc, bool move_trigger = true);
        int radCtrlHL(u_int8_t id, double rad, double speed, double acc, int currt_limit, bool move_trigger = true);
        void moveTrigger();

        // hub motor ctrl
        void hubMotorCtrl(int spd_1, int spd_2, int spd_3, int spd_4);

        // for applications: LyLinkArm
        int* getJointsZeroPosArray();
        void setJointsZeroPosArray(int values[]);
        int* getLinkArmPosSC();
        int* getLinkArmTorqueSC();
        void setCurrentSCPosMiddle();
        void linkArmSCJointsCtrlAngle(double angles[]);
        void linkArmSCJointsCtrlRad(double rads[]);

        void spaceIK2FPVIK();
        void FPVIK2spaceIK();
        bool linkArmPlaneIK(double x, double z);
        double* linkArmSpaceIK(double x, double y, double z, double g);
        double* linkArmFPVIK(double x, double b, double z, double g);

        double smoothCtrl(double start, double end, double rate);
        double getSmoothStepsXYZ(double x, double y, double z);
        double* smoothXYZGCtrl(double x, double y, double z, double g, double spd);
        double getSmoothStepsFPV(double r, double b, double z, double baseRate);
        double* smoothFPVAbsCtrl(double r, double b, double z, double g, double spd, double baseRate = 150.0);
        
        double* getJointFBRads();
        double* getJointGoalRads();
        double* getXYZGIK();
        double* getRBZGIK();
        void setMaxJointsSpeed(double speed);
        double getMaxJointsSpeed() { return jointsMaxSpeed; }
        void setLinkArmFeedbackFlag(bool flag, int hz);
        bool linkArmPlaneFK(double alpha, double beta, double& x, double& z);

        void ttlTestMachine();
    };

#endif