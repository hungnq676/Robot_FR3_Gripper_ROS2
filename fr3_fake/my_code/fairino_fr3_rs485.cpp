int testAxleGenCom()
{
  ROBOT_STATE_PKG pkg = {};
  FRRobot robot;
  robot.LoggerInit();
  robot.SetLoggerLevel(1);
  int rtn = robot.RPC("192.168.58.2");
  if (rtn != 0)
  {
    return -1;
  }
  robot.SetReConnectParam(true, 30000, 500);
  int led_on[6] = { 0xAB, 0xBA, 0x12, 0x01, 0x01, 0x79 };
  int led_off[6] = { 0xAB, 0xBA, 0x12, 0x01, 0x00, 0x78 };
  int version[5] = { 0xAB, 0xBA, 0x11, 0x00, 0x76 };
  int state[6] = { 0xAB, 0xBA, 0x1B, 0x01, 0xAA, 0x2B };
  int cycleState[6] = { 0xAB, 0xBA, 0x12, 0x01, 0x00, 0x78 };
  int rcvdata[16] = {0};
  int ret = 0;
  int cnt = 1;
  JointPos p1Joint(88.708, -86.178, 140.989, -141.825, -89.162, -49.879);
  DescPose p1Desc(188.007, -377.850, 260.207, 178.715, 2.823, -131.466);
  JointPos p2Joint(112.131, -75.554, 126.989, -139.027, -88.044, -26.477);
  DescPose p2Desc(368.003, -377.848, 260.211, 178.715, 2.823, -131.465);
  ExaxisPos exaxisPos(0, 0, 0, 0);
  DescPose offdese(0, 0, 0, 0, 0, 0);
  //Enable end-effector transparent transmission function
  robot.SetAxleGenComEnable(1);
  robot.SetAxleLuaEnable(1);
  while (cnt <= 10000)
  {
    //Read version number
    ret = robot.SndRcvAxleGenComCmdData(5, version, 10, rcvdata);
    printf(" hard version : %d,hard code:%d, soft version:%d %d, soft code:%d \n", rcvdata[4], rcvdata[5], rcvdata[6] ,rcvdata[7], rcvdata[8]);
    if (ret != 0)
    {
      break;
    }
    robot.Sleep(1000);
    //Read moxibustion head presence status
    ret = robot.SndRcvAxleGenComCmdData(6, state, 6, rcvdata);
    printf(" state : %d \n", rcvdata[4]);
    robot.Sleep(1000);
    //Turn on moxibustion head laser
    ret = robot.SndRcvAxleGenComCmdData(6, led_on, 6, rcvdata);
    printf("led on rcv data is: %d, %d, %d, %d, %d, %d  \n", rcvdata[0], rcvdata[1], rcvdata[2], rcvdata[3], rcvdata[4], rcvdata[5]);
    robot.MoveJ(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.Sleep(4000);
    //Turn off moxibustion head laser
    ret = robot.SndRcvAxleGenComCmdData(6, led_off, 6, rcvdata);
    printf("led off rcv data is: %d, %d, %d, %d, %d, %d \n", rcvdata[0], rcvdata[1], rcvdata[2], rcvdata[3], rcvdata[4], rcvdata[5]);
    robot.MoveJ(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.Sleep(1000);
    printf("***********************complate No. %d SDK test*****************************\n", cnt);
    cnt++;
  }
  robot.CloseRPC();
}