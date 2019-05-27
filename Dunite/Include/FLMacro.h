/********************************************************************
创建日期：	2003/06/17
产品名称：	DTFL静态库
模块名称：	flmacro
作者：		刘道余

功能说明：	常用的宏

===========================本模块历史修改记录==========================
============================历史修改记录完=============================
*********************************************************************/

#pragma once

//释放指针的宏
#define RELEASE(p) \
	if (p) \
	{ \
		delete p; \
		p = NULL; \
	}

// 程序中加说明，将在编译时在Build窗口显示相应语句
// 例如：在程序中用语句如：#pragma PROGMSG(以后再完善)，将在编译时在Build窗口显示此语句出处，双击就可到此处。
#define PROGSTR2(x)	#x
#define PROGSTR(x)	PROGSTR2(x)
#define PROGMSG(desc) message(__FILE__"("PROGSTR(__LINE__)"):"#desc)

