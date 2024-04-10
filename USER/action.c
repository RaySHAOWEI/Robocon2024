//
// Created by Ray on 2023/11/24.
//

#include "action.h"

ACTION_GL_POS ACTION_GL_POS_DATA = {0};

//更新action全场定位的值
void Update_Action_gl_position(float *value)
{
	//储存上一次的值
	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;

	//记录此次的值
	ACTION_GL_POS_DATA.ANGLE_Z = value[0]; // 有用。角度
	ACTION_GL_POS_DATA.ANGLE_X = value[1];
	ACTION_GL_POS_DATA.ANGLE_Y = value[2];
	ACTION_GL_POS_DATA.POS_X = value[3]; // 有用。x轴
	ACTION_GL_POS_DATA.POS_Y = value[4]; // 有用。y轴
	ACTION_GL_POS_DATA.W_Z = value[5];

	// 差分运算
	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;

	//偏航角直接赋值
	ROBOT_CHASSI.world_w = ACTION_GL_POS_DATA.ANGLE_Z;

	//累得出最终真实位置
	ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);
	ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y);//这里我根据车做了调整
	
	//变换到底盘中心
	ROBOT_CHASSI.world_x = ACTION_GL_POS_DATA.REAL_X-INSTALL_ERROR_X*cos(ROBOT_CHASSI.world_w*PI/180)+INSTALL_ERROR_Y*sin(ROBOT_CHASSI.world_w*PI/180)+INSTALL_ERROR_X;
	ROBOT_CHASSI.world_y = ACTION_GL_POS_DATA.REAL_Y-INSTALL_ERROR_X*sin(ROBOT_CHASSI.world_w*PI/180)-INSTALL_ERROR_Y*cos(ROBOT_CHASSI.world_w*PI/180)+INSTALL_ERROR_Y;
}
