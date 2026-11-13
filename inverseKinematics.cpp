#include <Servo.h>

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

//限制最大录制时长
const int MAX_RECORD = 200;
byte baseRecord[MAX_RECORD];
byte lArmRecord[MAX_RECORD];
byte rArmRecord[MAX_RECORD];
int recordIndex = 0;

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

//函数声明
void updateJoystick();      //摇杆控制
void servoCmd(int x, int y, int z);  //处理多个串口指令
void moveServo(Servo& servoName, int fromPos, int toPos);  //舵机运动
void handleSingleCmd(char c);        //处理单个转口指令
void OpenClaw();                     //打开机械钳
void CloseClaw();                    //合闭机械钳
void speedUp();                      //加快运行速度
void speedDown();                    //减慢运行速度
void startRecord();                  //开始录制
void stopRecord();                   //停止录制
void playRecord();                   //播放
void servoInit();                    //回中
void printServoInf();                //打印舵机状态
void pickA();                        //演示动作 A
void pickB();                        //演示动作 B
void pickC();                        //演示动作 C
bool isLegal(int x, int y, int z);   //判断是否合法
int getValue(String data, char key); //获取指令数据
bool inverseKinematics(float x, float y, float z, int& sBase, int& sLArm, int& sRArm);  //判断是否可以到达坐标
void moveTo(float x, float y, float z);  //控制机械臂到达坐标点
void drawLine(int fx,int fy,int fz,int tx,int ty,int tz);//回执直线
void drawTriangle(int xa,int ya,int za,int xb,int yb,int zb,int xc,int yc,int zc);//绘制三角形


void setup() {
	Serial.begin(9600);  // v2: 先开串口，初始化过程中的提示才能打印出来

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
	//摇杆只有在播放模式下不能使用（和绘制模式）
	if (state != PLAYING) updateJoystick();

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
		  写入录制数组中
		  if (recordIndex < MAX_RECORD) {
			baseRecord[recordIndex] = (byte)currentX;
			lArmRecord[recordIndex] = (byte)currentY;
			rArmRecord[recordIndex] = (byte)currentZ;
			recordIndex++;

			距离一秒时提醒
			if (recordIndex == MAX_RECORD - 20) {
			  Serial.println("+Reminder: 20 data points left, recording will stop soon.");
			}
		  } else {
			自动结束了录制
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


//舵机移动函数
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
	if(state == RECORDING) {
		if (c == 'R') stopRecord();
		return;
	}

    if(c == 'd'){
        mode++;
        mode %= 2;

        if(mode == 1){
            state = DRAWING;
            Serial.println("+Current State: DRAWING");
        }
        else{
            state = IDLE;
            Serial.println("+Current State: IDLE");
        }
        return;
    }

    if(state == IDLE){
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
    	    case 'P':
	        	playRecord();
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

    if(state == DRAWING){
        switch(c){
            case 'I':
	        	servoInit();
                break;
            case '1':
                drawLine(0,120,90,120,0,90);
                servoInit();
                break;
            case '2':
                drawTriangle(-60,60,60,0,120,60,60,60,60);
                servoInit();
                break;
            default:
                Serial.println("+Wraning: Unknown Command!");
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

		模式转变
		state = PLAYING;
		Serial.println("Start Playing...");

		读取数据点
		for (int i = 0; i < recordIndex; i++) {
		  base.write(baseRecord[i]);
		  lArm.write(lArmRecord[i]);
		  rArm.write(rArmRecord[i]);
		  delay(50);  //必须和写入数据的延迟一样，否则会少读数据点
		}

		换回来
		state = IDLE;
		Serial.println("End Playing...");

		让虚拟量与实际量相等，放拉回
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

//============================== 逆运动学（v2 重写） ==============================
//输入：目标点(x, y, z)，单位 mm
//+ X = 右、 + Y = 前、 + Z = 上（相对桌面的高度，肩关节在桌面上方 H 处）
//输出：sBase / sLArm / sRArm 三个舵机角度
//返回：true = 有解且三个舵机都在限位内；false = 不可达或超限

//几何关系（(r, d) 平面，r 为水平距离，d 为相对肩关节的高度差）：
//r = sqrt(x² + y²)
//d = z - H
//cos(theta3) = (r² + d² - L1² - L2²) / (2·L1·L2)     ← 必须用 d，不能用 z
//theta2 = atan2(L2·sin(theta3), L1 + L2·cos(theta3)) + atan2(d, r)
//theta1 = atan2(y, x)
bool inverseKinematics(float x, float y, float z, int& sBase, int& sLArm, int& sRArm) {
	float r = sqrt(x * x + y * y);     // 目标点到底座转轴的水平距离
	float d = z - H;                   // v2: 目标点相对肩关节的高度差（可正可负）
	float dist = sqrt(r * r + d * d);  // 肩关节到目标点的直线距离

	Serial.println("====================== NEED =====================");
	Serial.print(F(" x = "));
	Serial.print(x);
	Serial.print(F(" y = "));
	Serial.print(y);
	Serial.print(F(" z = "));
	Serial.println(z);
	Serial.print(F(" r = "));
	Serial.print(r);
	Serial.print(F(" d = "));
	Serial.print(d);
	Serial.print(F(" dist = "));
	Serial.println(dist);

	//可达性判断：肩到目标的距离必须落在[R_MIN, L1 + L2] 内
	if (dist > L1 + L2) {
		Serial.println(F("+Warning: Too far! dist > L1+L2"));
		Serial.println("=================================================");
		return false;
	}
	if (dist < R_MIN) {
		Serial.println(F("+Warning: Too close to the shoulder! dist < R_MIN"));
		Serial.println("=================================================");
		return false;
	}

	//余弦定理求肘关节内角 theta3（v2 : 用 d，不是 z）
	float cosT3 = (r * r + d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
	if (cosT3 > 1.0) cosT3 = 1.0;    // 数值保护
	if (cosT3 < -1.0) cosT3 = -1.0;
	float theta3 = acos(cosT3);

	//大臂相对水平面的夹角 theta2
	//v2 : d < 0 时 atan2(d, r) 自动为负，不需要再写 if 分支
	float phi = atan2(L2 * sin(theta3), L1 + L2 * cos(theta3));
	float theta2 = phi + atan2(d, r);

	//底座转角 theta1
	//v2 : base 舵机实测 0° = 右、90° = 正前、180° = 左，正好等于 theta1 的度数，无需偏移
	float theta1 = atan2(y, x);

	//映射到三个舵机的实际角度
	sBase = (int)lround(degrees(theta1));
	sLArm = (int)lround(LARM_OFFSET - degrees(theta3 - theta2));
	sRArm = (int)lround(RARM_OFFSET - degrees(theta2));

	Serial.print(F(" theta1 = "));
	Serial.print(theta1);
	Serial.print(F(" theta2 = "));
	Serial.print(theta2);
	Serial.print(F(" theta3 = "));
	Serial.println(theta3);
	Serial.print(F(" base = "));
	Serial.print(sBase);
	Serial.print(F(" lArm = "));
	Serial.print(sLArm);
	Serial.print(F(" rArm = "));
	Serial.println(sRArm);

    //v2: 限位检查改为返回值，超限时调用方不会去动舵机
    bool ok = true;
    if (sBase < baseMin || sBase > baseMax) {
    	Serial.println(F("+Warning: sBase is out of limits!"));
	    ok = false;
    }
    if (sLArm < lArmMin || sLArm > lArmMax) {
	    Serial.println(F("+Warning: sLArm is out of limits!"));
	    ok = false;
    }   
    if (sRArm < rArmMin || sRArm > rArmMax) {
	    Serial.println(F("+Warning: sRArm is out of limits!"));
	    ok = false;
    }
    Serial.println("=================================================");

    return ok;  // ★ v2: 原来缺了这一句；bool 函数掉出末尾 = 未定义行为
}

//控制机械臂到达世界坐标点(x, y, z)
void moveTo(float x, float y, float z) {
	int sBase, sLArm, sRArm;
	if (!inverseKinematics(x, y, z, sBase, sLArm, sRArm)) {
		Serial.println("+Warning: Can not Reach the Position!");
		state = IDLE;
		return;
	}

	int fromBase = base.read();
	int fromLArm = lArm.read();
	int fromRArm = rArm.read();

	Serial.print(F("Moving: base "));
	Serial.print(fromBase);
	Serial.print(F(" -> "));
	Serial.print(sBase);
	Serial.print(F("  lArm "));
	Serial.print(fromLArm);
	Serial.print(F(" -> "));
	Serial.print(sLArm);
	Serial.print(F("  rArm "));
	Serial.print(fromRArm);
	Serial.print(F(" -> "));
	Serial.println(sRArm);

	int step = 20;
	for (int i = 0; i <= step; i++) {
		base.write(fromBase + (sBase - fromBase) * i / step);
		lArm.write(fromLArm + (sLArm - fromLArm) * i / step);
		rArm.write(fromRArm + (sRArm - fromRArm) * i / step);
		delay(15);
	}

	currentX = sBase;
	currentY = sLArm;
	currentZ = sRArm;

	Serial.println(F("Done."));
}

void drawLine(int fx,int fy,int fz,int tx,int ty,int tz)
{
    moveTo(fx,fy,fz);
    moveTo(tx,ty,tz);
}

void drawTriangle(int xa,int ya,int za,int xb,int yb,int zb,int xc,int yc,int zc)
{
    drawLine(xa,ya,za,xb,yb,zb);
    drawLine(xb,yb,zb,xc,yc,zc);
    drawLine(xc,yc,zc,xa,ya,za);
}
