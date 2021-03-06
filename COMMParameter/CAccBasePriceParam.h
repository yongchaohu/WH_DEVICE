#pragma once
#include "CAccParamMaster.h"
#include "CParameterCommandExp.h"

// CAccBasePriceParam的唯一接口
#define theACC_BASEPRICE CAccBasePriceParam::GetInstance()        

/**
 @brief   ACC基础票价表参数类
*/

class PARAMETERCOMMAND_DECLSPEC CAccBasePriceParam :public CAccParamMaster
{
public: 

    static CAccBasePriceParam&  GetInstance()
    {
        return theInstance;
    }

protected:

private:

//	SECTION_INFO m_section[4];	//分段信息

    // 私有成员函数
    virtual void ParseBody(LPBYTE);
	virtual void ParseSectionInfo(LPBYTE);
	virtual bool CheckBodyInfo(LPBYTE);
	virtual void GetParamFileName();				 // 获取参数文件名


	CAccBasePriceParam(void);
    ~CAccBasePriceParam();
	CAccBasePriceParam(const CAccBasePriceParam&);
	CAccBasePriceParam& operator=(const CAccBasePriceParam&);

    static  CAccBasePriceParam theInstance;
};
