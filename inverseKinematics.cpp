/* ============================================================================
   inverseKinematics v4  ——  在 v3 基础上增加“手动示教点位 + 直线回放”
   （新文件，未修改 inverseKinematics.cpp / inverseKinematicsv3.cpp 及其他文件）
   ----------------------------------------------------------------------------
   继承 v3 的内容：
     1. drawLine() 是笛卡尔空间直线插补（1mm 采样 + 逐点逆解），末端走直线；
     2. 逆解输出 float 角度，用 Servo.writeMicroseconds() 驱动（约 0.09°/us）；
     3. 绘制(DRAWING)状态下停用摇杆，避免画画时被摇杆/悬空引脚拉走。
   v4 新增（本文件主要变化）：
     4. 手动示教：用摇杆把机械臂摆到纸上的点，按 Y 记录当前舵机状态，
        程序用正运动学把舵机角度换算成世界坐标存起来（最多 5 个点）；
     5. 按 E 清除所有已记录的点；
     6. 按 P 回放：先移动到第 1 个点，再依次用直线插补走到第 2、3…个点；
        回放前会先检查每一段直线是否全程可达，避免画到一半卡住；
     7. 按 T 打印已记录的点（调试用）。

   串口协议（9600，需要发送换行）：
     X<x>,Y<y>,Z<z>      在 DRAWING 模式下移动到世界坐标点，单位 mm
     d                    在 IDLE / DRAWING 之间切换
     Y                    记录当前舵机位置为下一个点（手动示教，最多 5 个）
     E                    清除所有记录点
     P                    依次用直线连接所有记录点并回放
     T                    打印已记录的点
     1                    画直线 (0,120,90)->(120,0,90)
     2                    画三角形 (-60,60,60) (0,120,60) (60,60,60)
     ?  I  O  S  H  L  V  A  B  C   同原版（V = 播放摇杆录制，原来的 P）
   世界坐标：+X=右、+Y=前、+Z=上（相对桌面，肩关节在桌面上方 H=60 处）
     base 舵机：0°=右、90°=正前、180°=左
   注意：点位数据存在 RAM 里，断电/复位会丢；需要掉电保存可以再加 EEPROM。
   ============================================================================ */

#include <Servo.h>
#include <math.h>

Servo claw, base, lArm, rArm;  //创建对象

//限制角度
const int baseMin = 0;
const int baseMax = 180;
const int rArmMin = 60;
const int rArmMax = 180;
const int lArmMin = 0;
const int lArmMax = 90;
const int clawMin = 0;
const int clawMax = 120;

//初始化引脚
const int basePin = A0;
const int lArmPin = A1;
const int rArmPin = A2;
const int clawPin = A3;

//限制最大录制时长（摇杆录制功能）
const int MAX_RECORD = 200;
byte baseRecord[MAX_RECORD];
byte lArmRecord[MAX_RECORD];
byte rArmRecord[MAX_RECORD];
int recordIndex = 0;

//---------------------- v4: 手动示教的点位 ----------------------
const int MAX_POINTS = 5;  //最多记录 5 个点（题目要求 3~5 个）

struct StoredPoint {
    float x, y, z;   //世界坐标（mm），由记录时的舵机角度做正运动学算出
    int sb, sl, sr;  //记录时的舵机角度（方便查看/调试）
};

StoredPoint points[MAX_POINTS];
int pointCount = 0;

//限制系统状态：空闲、录制、播放、绘画
enum State {
    IDLE,
    RECORDING,
    PLAYING,
    DRAWING
};
//初始化状态
State state = IDLE;

//摇杆中值
const int JOY_CENTER = 512;
//摇杆死区
const int JOY_DEADZONE = 50;  //死区，防止抖动

//每个舵机的当前角度（增量控制的“状态”）
int currentX = 90;
int currentY = 90;
int currentZ = 90;
int currentG = 90;

//移动速度系数，越大越慢
int SPEED_DIV = 150;

//延迟时长
int DSD = 15;

int steps = 0;
int mode = 0;

//舵机安装偏移（需要按实物标定）
//v2 : base 舵机不需要偏移 —— 实测 0° = 右方(+X)、90° = 正前方(+Y)、180° = 左方(-X)，
//sBase 直接等于 theta1 的度数，所以删掉了原来一直没用上的 BASE_OFFSET。
const float LARM_OFFSET = 90.0;
const float RARM_OFFSET = 180.0;

//单位mm
const float L1 = 80.0;     //大臂(rArm)
const float L2 = 80.0;     //小臂(lArm)
const float H = 60.0;      //肩部离桌面高度
const float R_MIN = 40.0;  //肩关节到目标点的最小距离（避免肘关节完全折叠的奇异位形）

//attach(pin, 500, 2500) 时，舵机 0~180° 对应 500~2500us
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const float SERVO_DEG_PER_US = 180.0 / (SERVO_MAX_US - SERVO_MIN_US);  // 0.09°/us

//---------------------- 画线参数（v3 新增） ----------------------
//直线插补步长：越小越直，但点更多、更慢。1mm 时 1° 舵机分辨率是主要误差
const float LINE_STEP_MM = 1.0;
//每个插补点之间的延时：决定画线速度，也影响舵机跟随误差
const int LINE_DELAY_MS = 20;
//单条线的最大采样点数，防止参数写错时卡死
const int LINE_MAX_POINTS = 400;

//角度比较的容差（避免 179.9999 这种边界值被误判超限）
const float LIMIT_EPS = 0.01;

//函数声明
void updateJoystick();      //摇杆控制
void servoCmd(int x, int y, int z);  //处理多个串口指令
void moveServo(Servo& servoName, int fromPos, int toPos);  //舵机运动
void servoWriteDeg(Servo& servoName, float deg);  //按浮点角度写舵机（v3 新增）
void handleSingleCmd(char c);        //处理单个转口指令
void OpenClaw();                     //打开机械钳
void CloseClaw();                    //合闭机械钳
void speedUp();                      //加快运行速度
void speedDown();                    //减慢运行速度
void startRecord();                  //开始录制
void stopRecord();                   //停止录制
void playRecord();                   //播放摇杆录制（v4: 按键改为 V）
void servoInit();                    //回中
void printServoInf();                //打印舵机状态
void pickA();                        //演示动作 A
void pickB();                        //演示动作 B
void pickC();                        //演示动作 C
bool isLegal(int x, int y, int z);   //判断是否合法
int getValue(String data, char key); //获取指令数据
bool inverseKinematics(float x, float y, float z, float& sBase, float& sLArm, float& sRArm, bool verbose);  //逆解
bool moveTo(float x, float y, float z);  //点对点移动（关节空间插值）
bool drawLine(float fx, float fy, float fz, float tx, float ty, float tz, bool goToStart = true);  //直线插补（v4: 返回是否画完，可选是否先走到起点）
void drawTriangle(float xa, float ya, float za, float xb, float yb, float zb, float xc, float yc, float zc);
void forwardKinematics(float sb, float sl, float sr, float& x, float& y, float& z);  //v4: 舵机角度 -> 世界坐标
void recordPoint();        //v4: 记录当前点位（Y）
void clearPoints();        //v4: 清除点位（E）
void printPoints();        //v4: 打印点位（T）
bool segmentReachable(const StoredPoint& p0, const StoredPoint& p1);  //v4: 检查一段直线是否全程可达
void playTrack();          //v4: 依次直线回放所有点位（P）

void setup() {
    Serial.begin(9600);

    //初始化舵机
    base.attach(9, 500, 2500);
    delay(500);
    lArm.attach(8, 500, 2500);
    delay(500);
    rArm.attach(7, 500, 2500);
    delay(500);
    claw.attach(6, 500, 2500);
    delay(500);

    servoInit();  //回中

    Serial.println("Please Input Data: ");
}

void loop() {
    //摇杆只有在播放模式和绘制模式下不能使用
    //v3: 加了 DRAWING 的排除，否则画线时（以及画完后）悬空/误碰的摇杆会把机械臂拉走
    if (state != PLAYING && state != DRAWING) updateJoystick();

    //检测是否有串口数据，并处理
    if (Serial.available() > 0) {
        String serialCmd = Serial.readStringUntil('\n');  //截取整行指令
        serialCmd.trim();                                 //删除空格、换行符

        if (serialCmd.length() == 0) return;  //防误触

        if (serialCmd.length() == 1)  //处理单指令
        {
            handleSingleCmd(serialCmd[0]);
            return;  //执行一次直接结束，避免后续二次执行。
        }

        //串口控制，支持混合指令
        if (state == IDLE && (serialCmd.indexOf('x') != -1 || serialCmd.indexOf('y') != -1 || serialCmd.indexOf('z') != -1 || serialCmd.indexOf('g') != -1)) {
            //x, y, z, g分别对应base, lArm, rArm, claw
            //三元运算
            int x = (serialCmd.indexOf('x') != -1) ? getValue(serialCmd, 'x') : base.read();
            int y = (serialCmd.indexOf('y') != -1) ? getValue(serialCmd, 'y') : lArm.read();
            int z = (serialCmd.indexOf('z') != -1) ? getValue(serialCmd, 'z') : rArm.read();
            int g = (serialCmd.indexOf('g') != -1) ? getValue(serialCmd, 'g') : claw.read();

            //检测指令是否合法
            if (isLegal(x, y, z)) servoCmd(x, y, z);
            else Serial.println("+Warning: Your Command is Out Of Limits!");

            //单独判断claw指令
            if (g >= clawMin && g <= clawMax) {
                moveServo(claw, claw.read(), g);
                currentG = g;  // 同步状态，防止被摇杆拉回去
                Serial.print("claw -> ");
                Serial.println(g);
            }
            else {
                Serial.println("+Warning: Claw Angle Out Of Limits!");
            }
        }
        else if (state == DRAWING && (serialCmd.indexOf('X') != -1 && serialCmd.indexOf('Y') != -1 && serialCmd.indexOf('Z') != -1)) {
            int X = getValue(serialCmd, 'X');
            int Y = getValue(serialCmd, 'Y');
            int Z = getValue(serialCmd, 'Z');

            moveTo(X, Y, Z);
        }
        else {
            Serial.println("+Warning: Unknown Command!");
        }
    }
}

void updateJoystick()  //摇杆控制核心
{
    //读取数据
    int rawX = analogRead(basePin);
    int rawY = analogRead(lArmPin);
    int rawZ = analogRead(rArmPin);
    int rawG = analogRead(clawPin);

    //减去半值（中心值为512）
    int offsetX = rawX - JOY_CENTER;
    int offsetY = rawY - JOY_CENTER;
    int offsetZ = rawZ - JOY_CENTER;
    int offsetG = rawG - JOY_CENTER;

    //死区过滤，防止摇杆微小位移而发生抖动，确定需要多大的幅度才能使机械臂移动
    if (abs(offsetX) < JOY_DEADZONE) offsetX = 0;
    if (abs(offsetY) < JOY_DEADZONE) offsetY = 0;
    if (abs(offsetZ) < JOY_DEADZONE) offsetZ = 0;
    if (abs(offsetG) < JOY_DEADZONE) offsetG = 0;

    //累计变量
    currentX -= offsetX / SPEED_DIV;
    currentY -= offsetY / SPEED_DIV;
    currentZ -= offsetZ / SPEED_DIV;
    currentG += offsetG / SPEED_DIV;

    //限幅
    currentX = constrain(currentX, baseMin, baseMax);
    currentY = constrain(currentY, lArmMin, lArmMax);
    currentZ = constrain(currentZ, rArmMin, rArmMax);
    currentG = constrain(currentG, clawMin, clawMax);

    //写入
    base.write(currentX);
    lArm.write(currentY);
    rArm.write(currentZ);
    claw.write(currentG);

    //顺带录制
    if (state == RECORDING) {
        if (recordIndex < MAX_RECORD) {
            baseRecord[recordIndex] = (byte)currentX;
            lArmRecord[recordIndex] = (byte)currentY;
            rArmRecord[recordIndex] = (byte)currentZ;
            recordIndex++;

            //快录满时提醒
            if (recordIndex == MAX_RECORD - 20) {
                Serial.println("+Reminder: 20 data points left, recording will stop soon.");
            }
        }
        else {
            //自动结束了录制
            stopRecord();
            Serial.println("+Warning: Record buffer full! Auto stopped.");
        }
    }

    //50帧每秒记录一次数据
    delay(50);
}

//处理串口指令
void servoCmd(int x, int y, int z) {
    Serial.print("Receive Command:x = ");
    Serial.print(x);
    Serial.print(" y = ");
    Serial.print(y);
    Serial.print(" z = ");
    Serial.println(z);

    moveServo(base, base.read(), x);
    moveServo(lArm, lArm.read(), y);
    moveServo(rArm, rArm.read(), z);

    //只要涉及直接写入舵机的就要手动统一数据
    currentX = x;
    currentY = y;
    currentZ = z;
}

//舵机移动函数（整数角度，逐个角度移动，用于演示动作）
void moveServo(Servo& servoName, int fromPos, int toPos) {
    if (fromPos > toPos) {
        for (int i = fromPos; i >= toPos; i--) {
            servoName.write(i);
            delay(DSD);
        }
    }
    else {
        for (int i = fromPos; i <= toPos; i++) {
            servoName.write(i);
            delay(DSD);
        }
    }
}

//按浮点角度写舵机（v3 新增）：用 us 直接驱动，分辨率约 0.09°
void servoWriteDeg(Servo& servoName, float deg) {
    int us = (int)lround(SERVO_MIN_US + deg / SERVO_DEG_PER_US);
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    servoName.writeMicroseconds(us);
}

//获取数值
int getValue(String data, char key) {
    int index = data.indexOf(key);
    if (index == -1) return -1;

    int end = data.indexOf(',', index);  //x45，y45
    if (end == -1) end = data.length();  //针对单舵机指令

    return data.substring(index + 1, end).toInt();
}

//判断指令是否合法
bool isLegal(int x, int y, int z) {
    if (x < baseMin || x > baseMax) return false;
    if (y < lArmMin || y > lArmMax) return false;
    if (z < rArmMin || z > rArmMax) return false;
    return true;
}

//处理单字符指令
void handleSingleCmd(char c) {
    if (state == RECORDING) {
        if (c == 'R') stopRecord();
        return;
    }

    if (c == 'd') {
        mode++;
        mode %= 2;

        if (mode == 1) {
            state = DRAWING;
            Serial.println("+Current State: DRAWING");
        }
        else {
            state = IDLE;
            Serial.println("+Current State: IDLE");
        }
        return;
    }

    //v4: 点位示教/回放（IDLE 和 DRAWING 下都可用，不改变当前状态）
    if (c == 'Y') {
        recordPoint();
        return;
    }
    if (c == 'E') {
        clearPoints();
        return;
    }
    if (c == 'P') {
        playTrack();
        return;
    }
    if (c == 'T') {
        printPoints();
        return;
    }

    if (state == IDLE) {
        switch (c) {
            case '?':
                Serial.println("----- Servo Position -----");
                printServoInf();
                Serial.println("--------------------------");
                break;
            case 'U':
                if (steps == 0) {
                    Serial.println("Playing A.");
                    pickA();
                }
                else if (steps == 1) {
                    Serial.println("Playing B.");
                    pickB();
                }
                else {
                    Serial.println("Playing C.");
                    pickC();
                }
                steps++;
                steps = steps % 3;
                break;
            case 'I':
                servoInit();
                break;
            case 'O':
                OpenClaw();
                break;
            case 'S':
                CloseClaw();
                break;
            case 'H':
                speedUp();
                break;
            case 'L':
                speedDown();
                break;
            case 'R':
                startRecord();
                break;
            case 'V':
                playRecord();  //v4: 摇杆录制的播放改为 V（P 留给点位回放）
                break;
            case 'A':
                pickA();
                break;
            case 'B':
                pickB();
                break;
            case 'C':
                pickC();
                break;
            default:
                Serial.println("+Warning:unknown command!");
                break;
        }
    }

    if (state == DRAWING) {
        switch (c) {
            case 'I':
                servoInit();
                break;
            case '1':
                drawLine(0, 120, 90, 120, 0, 90);
                servoInit();
                break;
            case '2':
                drawTriangle(-60, 60, 60, 0, 120, 60, 60, 60, 60);
                servoInit();
                break;
            default:
                Serial.println("+Warning: Unknown Command!");
        }
    }
}

//录制
void startRecord() {
    state = RECORDING;
    recordIndex = 0;
    Serial.println("Start recording...");
}

//停止
void stopRecord() {
    state = IDLE;
    Serial.print("End Recording, with a total of  ");
    Serial.print(recordIndex);
    Serial.println(" data points.");
}

//播放
void playRecord() {
    //特殊情况，防止误触
    if (recordIndex == 0) {
        Serial.println("+Warning:No Recording Data!");
        return;
    }

    //模式转变
    state = PLAYING;
    Serial.println("Start Playing...");

    //读取数据点
    for (int i = 0; i < recordIndex; i++) {
        base.write(baseRecord[i]);
        lArm.write(lArmRecord[i]);
        rArm.write(rArmRecord[i]);
        delay(50);  //必须和写入数据的延迟一样，否则会少读数据点
    }

    //换回来
    state = IDLE;
    Serial.println("End Playing...");

    //让虚拟量与实际量相等，防拉回
    if (recordIndex > 0) {
        currentX = baseRecord[recordIndex - 1];
        currentY = lArmRecord[recordIndex - 1];
        currentZ = rArmRecord[recordIndex - 1];
    }
}

//打开钳子
void OpenClaw() {
    moveServo(claw, claw.read(), clawMin);
    currentG = clawMin;
}

//关上钳子
void CloseClaw() {
    moveServo(claw, claw.read(), clawMax);
    currentG = clawMax;
}

//加速
void speedUp() {
    if (SPEED_DIV > 40) SPEED_DIV -= 20;
    if (DSD > 4) DSD -= 2;

    Serial.print("Speed UP: SPEED_DIV=");
    Serial.print(SPEED_DIV);
    Serial.print(", DSD=");
    Serial.println(DSD);
}

//减速
void speedDown() {
    if (SPEED_DIV < 300) SPEED_DIV += 20;
    else {
        Serial.print("+Warning: Too Slow! SPEED_DIV = ");
        Serial.println(SPEED_DIV);
        return;
    }

    if (DSD < 40) DSD += 2;
    else {
        Serial.print("+Warning: Too Slow! DSD = ");
        Serial.println(SPEED_DIV);
        return;
    }

    Serial.print("Speed DOWN: SPEED_DIV=");
    Serial.print(SPEED_DIV);
    Serial.print(", DSD=");
    Serial.println(DSD);
}

//回中（初始化）
void servoInit() {
    servoCmd(90, 60, 90);
    currentX = 90;
    currentY = 60;
    currentZ = 90;

    delay(300);

    moveServo(claw, claw.read(), 90);
    currentG = 90;
}

//打印信息
void printServoInf() {
    Serial.println("===========Servo information============");
    Serial.print("base: read=");
    Serial.print(base.read());
    Serial.print("  current=");
    Serial.print(currentX);

    Serial.print("  lArm: read=");
    Serial.print(lArm.read());
    Serial.print(" current=");
    Serial.print(currentY);

    Serial.print("  rArm: read=");
    Serial.print(rArm.read());
    Serial.print(" current=");
    Serial.print(currentZ);

    Serial.print("  claw: read=");
    Serial.print(claw.read());
    Serial.print(" current=");
    Serial.print(currentG);

    Serial.print("  SPEED_DIC: ");
    Serial.print(SPEED_DIV);
    Serial.print("  DSD: ");
    Serial.println(DSD);

    //v4: 顺带显示已记录的点数
    Serial.print(F("  points: "));
    Serial.print(pointCount);
    Serial.print(F("/"));
    Serial.println(MAX_POINTS);

    Serial.print(" State=");
    Serial.println(state);

    Serial.println("===================================");
}

void pickA()  //铅笔
{
    servoInit();
    moveServo(claw, claw.read(), 45);
    currentG = 45;
    delay(1000);
    moveServo(rArm, rArm.read(), 130);
    currentZ = 130;
    delay(1000);
    moveServo(claw, claw.read(), 97);
    currentG = 97;
    delay(1000);
    moveServo(rArm, rArm.read(), 90);
    currentZ = 90;
    delay(1000);
    moveServo(base, base.read(), 0);
    currentX = 0;
    delay(1000);
    moveServo(lArm, lArm.read(), 18);
    currentY = 18;
    delay(1000);
    moveServo(claw, claw.read(), 45);
    currentG = 45;
    delay(1000);
    servoInit();
    delay(1000);
}

void pickB()  //coin
{
    servoInit();
    moveServo(claw, claw.read(), 50);
    currentG = 50;
    delay(1000);
    moveServo(rArm, rArm.read(), 130);
    currentZ = 130;
    delay(1000);
    moveServo(claw, claw.read(), 107);
    currentG = 107;
    delay(1000);
    moveServo(rArm, rArm.read(), 90);
    currentZ = 90;
    delay(1000);
    moveServo(base, base.read(), 180);
    currentX = 180;
    delay(1000);
    moveServo(lArm, lArm.read(), 18);
    currentY = 18;
    delay(1000);
    moveServo(claw, claw.read(), 50);
    currentG = 50;
    delay(1000);
    servoInit();
    delay(1000);
}

void pickC()  //不规则物体(钥匙)
{
    servoInit();
    delay(1000);
    moveServo(claw, claw.read(), 45);
    currentG = 45;
    delay(1000);
    moveServo(lArm, lArm.read(), 70);
    currentY = 70;
    delay(1000);
    moveServo(rArm, rArm.read(), 130);
    currentZ = 130;
    delay(1000);
    moveServo(claw, claw.read(), 100);
    currentG = 100;
    delay(1000);
    moveServo(rArm, rArm.read(), 90);
    currentZ = 90;
    delay(1000);
    moveServo(lArm, lArm.read(), 40);
    currentY = 40;
    delay(1000);
    moveServo(rArm, rArm.read(), 130);
    currentZ = 130;
    delay(1000);
    moveServo(claw, claw.read(), 45);
    currentG = 45;
    delay(1000);
    servoInit();
}

//============================== 逆运动学 ==============================
//输入：目标点(x, y, z)，单位 mm
//  +X = 右、+Y = 前、+Z = 上（相对桌面的高度，肩关节在桌面上方 H 处）
//输出：sBase / sLArm / sRArm 三个舵机角度（float，度）
//返回：true = 有解且三个舵机都在限位内；false = 不可达或超限
//verbose：true 时打印整段 NEED 调试信息；画线时传 false（否则 9600 波特率会被打印拖慢）
//
//几何关系（(r, d) 平面，r 为水平距离，d 为相对肩关节的高度差）：
//  r = sqrt(x² + y²)
//  d = z - H
//  cos(theta3) = (r² + d² - L1² - L2²) / (2·L1·L2)     ← 必须用 d，不能用 z
//  theta3 = 180° - 三角形内角 = 肘关节外角 = 小臂相对大臂的转角
//           （0 = 两杆伸直，180° = 完全折叠；小臂绝对方向角 gamma = theta2 - theta3）
//  theta2 = atan2(L2·sin(theta3), L1 + L2·cos(theta3)) + atan2(d, r)
//  theta1 = atan2(y, x)
bool inverseKinematics(float x, float y, float z, float& sBase, float& sLArm, float& sRArm, bool verbose) {
    float r = sqrt(x * x + y * y);     // 目标点到底座转轴的水平距离
    float d = z - H;                   // 目标点相对肩关节的高度差（可正可负）
    float dist = sqrt(r * r + d * d);  // 肩关节到目标点的直线距离

    if (verbose) {
        Serial.println("====================== NEED =====================");
        Serial.print(F(" x = "));    Serial.print(x);
        Serial.print(F(" y = "));    Serial.print(y);
        Serial.print(F(" z = "));    Serial.println(z);
        Serial.print(F(" r = "));    Serial.print(r);
        Serial.print(F(" d = "));    Serial.print(d);
        Serial.print(F(" dist = ")); Serial.println(dist);
    }

    //可达性判断：肩到目标的距离必须落在 [R_MIN, L1 + L2] 内
    if (dist > L1 + L2) {
        if (verbose) {
            Serial.println(F("+Warning: Too far! dist > L1+L2"));
            Serial.println("=================================================");
        }
        return false;
    }
    if (dist < R_MIN) {
        if (verbose) {
            Serial.println(F("+Warning: Too close to the shoulder! dist < R_MIN"));
            Serial.println("=================================================");
        }
        return false;
    }

    //余弦定理求肘关节外角 theta3（= 小臂相对大臂的转角：0=伸直，π=完全折叠）
    float cosT3 = (r * r + d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    if (cosT3 > 1.0) cosT3 = 1.0;    // 数值保护
    if (cosT3 < -1.0) cosT3 = -1.0;
    float theta3 = acos(cosT3);

    //大臂相对水平面的夹角 theta2：d < 0 时 atan2(d, r) 自动为负，不需要额外分支
    float phi = atan2(L2 * sin(theta3), L1 + L2 * cos(theta3));
    float theta2 = phi + atan2(d, r);

    //底座转角 theta1：base 舵机 0°=右、90°=正前、180°=左，正好等于 theta1 的度数
    float theta1 = atan2(y, x);

    //映射到三个舵机的实际角度（float，不用取整，交给 writeMicroseconds）
    sBase = degrees(theta1);
    sLArm = LARM_OFFSET - degrees(theta3 - theta2);
    sRArm = RARM_OFFSET - degrees(theta2);

    if (verbose) {
        Serial.print(F(" theta1 = ")); Serial.print(theta1);
        Serial.print(F(" theta2 = ")); Serial.print(theta2);
        Serial.print(F(" theta3 = ")); Serial.println(theta3);
        Serial.print(F(" base = "));   Serial.print(sBase, 2);
        Serial.print(F(" lArm = "));   Serial.print(sLArm, 2);
        Serial.print(F(" rArm = "));   Serial.println(sRArm, 2);
    }

    //限位检查：超限时返回 false，调用方不会去动舵机
    bool ok = true;
    if (sBase < baseMin - LIMIT_EPS || sBase > baseMax + LIMIT_EPS) {
        if (verbose) Serial.println(F("+Warning: sBase is out of limits!"));
        ok = false;
    }
    if (sLArm < lArmMin - LIMIT_EPS || sLArm > lArmMax + LIMIT_EPS) {
        if (verbose) Serial.println(F("+Warning: sLArm is out of limits!"));
        ok = false;
    }
    if (sRArm < rArmMin - LIMIT_EPS || sRArm > rArmMax + LIMIT_EPS) {
        if (verbose) Serial.println(F("+Warning: sRArm is out of limits!"));
        ok = false;
    }

    if (verbose) Serial.println("=================================================");
    return ok;
}

//点对点移动：两个点之间走关节空间插值（不要求直线，动作更自然、更快）
bool moveTo(float x, float y, float z) {
    float tB, tL, tR;
    if (!inverseKinematics(x, y, z, tB, tL, tR, true)) {
        Serial.println("+Warning: Can not Reach the Position!");
        return false;
    }

    //当前角度（整数即可，作为插值起点）
    float fB = base.read();
    float fL = lArm.read();
    float fR = rArm.read();

    const int step = 25;
    for (int i = 0; i <= step; i++) {
        float u = (float)i / step;
        servoWriteDeg(base, fB + (tB - fB) * u);
        servoWriteDeg(lArm, fL + (tL - fL) * u);
        servoWriteDeg(rArm, fR + (tR - fR) * u);
        delay(15);
    }

    currentX = (int)lround(tB);
    currentY = (int)lround(tL);
    currentZ = (int)lround(tR);

    return true;
}

//============================== 直线插补（v3 核心 / v4 增加返回值） ==============================
//把世界坐标里的直线拆成很多小段：每一小段都逆解，再直接把三个舵机写到该点。
//这样末端走的是直线，而不是“关节空间插值”形成的圆弧。
//goToStart = true：先 moveTo 到起点（单条线用）；
//goToStart = false：调用者保证已经在起点（连续多段回放用，省掉重复的起手动作）。
bool drawLine(float fx, float fy, float fz, float tx, float ty, float tz, bool goToStart) {
    //1) 先检查两个端点是否可达（避免白跑一趟）
    float a, b, c;
    if (!inverseKinematics(fx, fy, fz, a, b, c, false) ||
        !inverseKinematics(tx, ty, tz, a, b, c, false)) {
        Serial.println("+Warning: Endpoint unreachable, line skipped.");
        return false;
    }

    //2) 先走到起点（这一段是关节空间移动，不属于画出来的线）
    if (goToStart && !moveTo(fx, fy, fz)) return false;

    //3) 按 LINE_STEP_MM 把直线拆成 n 段
    float dx = tx - fx;
    float dy = ty - fy;
    float dz = tz - fz;
    float len = sqrt(dx * dx + dy * dy + dz * dz);

    int n = (int)ceil(len / LINE_STEP_MM);
    if (n < 1) n = 1;
    if (n > LINE_MAX_POINTS) n = LINE_MAX_POINTS;

    Serial.print(F("Drawing line, points = "));
    Serial.println(n + 1);

    //4) 逐点逆解并写舵机（i=0 就是起点，已经到位，所以从 1 开始）
    for (int i = 1; i <= n; i++) {
        float u = (float)i / (float)n;
        float x = fx + dx * u;
        float y = fy + dy * u;
        float z = fz + dz * u;

        float sB, sL, sR;
        if (!inverseKinematics(x, y, z, sB, sL, sR, false)) {
            //中间某个点不可达：停下来并提示，避免机械臂乱甩
            Serial.print(F("+Warning: point out of reach, i = "));
            Serial.println(i);
            return false;
        }

        servoWriteDeg(base, sB);
        servoWriteDeg(lArm, sL);
        servoWriteDeg(rArm, sR);

        //同步摇杆的“虚拟角度”，防止画完后被摇杆拉回
        currentX = (int)lround(sB);
        currentY = (int)lround(sL);
        currentZ = (int)lround(sR);

        delay(LINE_DELAY_MS);
    }

    Serial.println(F("Line done."));
    return true;
}

void drawTriangle(float xa, float ya, float za, float xb, float yb, float zb, float xc, float yc, float zc) {
    drawLine(xa, ya, za, xb, yb, zb);
    drawLine(xb, yb, zb, xc, yc, zc);
    drawLine(xc, yc, zc, xa, ya, za);
}

//============================== v4: 手动示教点位 + 直线回放 ==============================
//正运动学：已知三个舵机角度，求末端世界坐标
//（用于把“摇杆手动摆到的位置”变成世界坐标存起来）
//  theta1 = sBase
//  theta2 = RARM_OFFSET - sRArm
//  gamma  = sLArm - LARM_OFFSET      // 小臂的绝对方向角 = theta2 - theta3
//                                    //（theta3 是肘部外角，即小臂相对大臂的转角）
//  r = L1*cos(theta2) + L2*cos(gamma)
//  d = L1*sin(theta2) + L2*sin(gamma)
//  x = r*cos(theta1), y = r*sin(theta1), z = d + H
void forwardKinematics(float sb, float sl, float sr, float& x, float& y, float& z) {
    float t1 = radians(sb);
    float t2 = radians(RARM_OFFSET - sr);
    float gamma = radians(sl - LARM_OFFSET);
    float r = L1 * cos(t2) + L2 * cos(gamma);
    float d = L1 * sin(t2) + L2 * sin(gamma);
    x = r * cos(t1);
    y = r * sin(t1);
    z = d + H;
}

//按 Y：记录当前舵机位置为下一个点
void recordPoint() {
    if (pointCount >= MAX_POINTS) {
        Serial.print(F("+Warning: already "));
        Serial.print(MAX_POINTS);
        Serial.println(F(" points, press E to clear."));
        return;
    }

    //记录当前“下发”的舵机角度（摇杆控制时就是机械臂现在的位置）
    int sb = base.read();
    int sl = lArm.read();
    int sr = rArm.read();

    if (!isLegal(sb, sl, sr)) {
        Serial.println(F("+Warning: current pose out of servo limits, not recorded."));
        return;
    }

    //正解得到世界坐标
    float x, y, z;
    forwardKinematics(sb, sl, sr, x, y, z);

    //base 在 180° 极限位置时，float 的 sin(pi) 会得到极小的负数，
    //同一个点会被算到 -y 一侧（回放时 base 解出 -180 而超限），这里把极限方向上的微小 y 归零。
    if (fabs(y) < 0.01f) y = 0.0f;

    //反解验证：保证这个点回放时一定能解出来
    float tb, tl, tr;
    if (!inverseKinematics(x, y, z, tb, tl, tr, false)) {
        Serial.println(F("+Warning: recorded pose cannot be solved back, not recorded."));
        return;
    }

    //镜像位形检查：如果末端在同一个点、但手臂是“反向折叠”的（正解再反解回不到原角度），
    //回放时 base 会转到相反方向、臂形完全不同，所以直接拒绝，避免回放时突然翻臂。
    //正常位形下正解再反解是精确的（只有浮点误差，diff 约 0.00x°）；
    //镜像位形的角度差通常在 100° 以上，所以 10° 是一个两边都留足余量的经验阈值。
    //注意：base 角是“圆形”的，-180° 与 +180° 是同一个位置（float 在 180° 处会算出 -180），
    //      所以要先做角度回绕，否则最左边（base=180°）的正常点位会被误判。
    float dBase = fabs(tb - sb);
    if (dBase > 180.0) dBase = 360.0 - dBase;
    float diff = dBase + fabs(tl - sl) + fabs(tr - sr);
    if (diff > 10.0) {
        Serial.println(F("+Warning: mirrored arm pose, move the arm to the normal side and retry."));
        return;
    }

    points[pointCount].x = x;
    points[pointCount].y = y;
    points[pointCount].z = z;
    points[pointCount].sb = sb;
    points[pointCount].sl = sl;
    points[pointCount].sr = sr;
    pointCount++;

    Serial.print(F("Point "));
    Serial.print(pointCount);
    Serial.print(F("/"));
    Serial.print(MAX_POINTS);
    Serial.print(F(" saved: servo("));
    Serial.print(sb);
    Serial.print(F(","));
    Serial.print(sl);
    Serial.print(F(","));
    Serial.print(sr);
    Serial.print(F(") -> world("));
    Serial.print(x, 1);
    Serial.print(F(", "));
    Serial.print(y, 1);
    Serial.print(F(", "));
    Serial.print(z, 1);
    Serial.println(F(")"));
}

//按 E：清除所有点
void clearPoints() {
    pointCount = 0;
    Serial.println(F("All points cleared."));
}

//按 T：打印已记录的点
void printPoints() {
    if (pointCount == 0) {
        Serial.println(F("No points recorded."));
        return;
    }
    for (int i = 0; i < pointCount; i++) {
        Serial.print(F("Point "));
        Serial.print(i + 1);
        Serial.print(F(": servo("));
        Serial.print(points[i].sb);
        Serial.print(F(","));
        Serial.print(points[i].sl);
        Serial.print(F(","));
        Serial.print(points[i].sr);
        Serial.print(F(") world("));
        Serial.print(points[i].x, 1);
        Serial.print(F(", "));
        Serial.print(points[i].y, 1);
        Serial.print(F(", "));
        Serial.print(points[i].z, 1);
        Serial.println(F(")"));
    }
}

//检查 p0 -> p1 这条直线上的每个采样点是否都能逆解出来（只计算，不动机器）
bool segmentReachable(const StoredPoint& p0, const StoredPoint& p1) {
    float dx = p1.x - p0.x;
    float dy = p1.y - p0.y;
    float dz = p1.z - p0.z;
    float len = sqrt(dx * dx + dy * dy + dz * dz);

    int n = (int)ceil(len / LINE_STEP_MM);
    if (n < 1) n = 1;
    if (n > LINE_MAX_POINTS) n = LINE_MAX_POINTS;

    for (int i = 0; i <= n; i++) {
        float u = (float)i / (float)n;
        float sb, sl, sr;
        if (!inverseKinematics(p0.x + dx * u, p0.y + dy * u, p0.z + dz * u, sb, sl, sr, false)) {
            return false;
        }
    }
    return true;
}

//按 P：从第 1 个点出发，依次用直线连到后面的点
void playTrack() {
    if (pointCount < 2) {
        Serial.println(F("+Warning: need at least 2 points. Move arm and press Y."));
        return;
    }

    //先检查所有直线段，避免画到一半才发现某段够不着
    for (int i = 1; i < pointCount; i++) {
        if (!segmentReachable(points[i - 1], points[i])) {
            Serial.print(F("+Warning: segment "));
            Serial.print(i);
            Serial.println(F(" is not fully reachable, playback cancelled."));
            return;
        }
    }

    State saved = state;
    state = PLAYING;  //回放期间停用摇杆
    Serial.print(F("Playing "));
    Serial.print(pointCount);
    Serial.println(F(" points..."));

    //先移动到第 1 个点（这一段不要求直线）
    if (!moveTo(points[0].x, points[0].y, points[0].z)) {
        Serial.println(F("+Warning: cannot move to point 1, playback cancelled."));
        state = saved;
        return;
    }

    //依次直线连接后面的点
    for (int i = 1; i < pointCount; i++) {
        Serial.print(F("Segment "));
        Serial.print(i);
        Serial.print(F(" -> "));
        Serial.println(i + 1);
        if (!drawLine(points[i - 1].x, points[i - 1].y, points[i - 1].z,
                      points[i].x, points[i].y, points[i].z, false)) {
            Serial.println(F("+Warning: playback stopped."));
            break;
        }
    }

    state = saved;
    Serial.println(F("Play done."));
}
