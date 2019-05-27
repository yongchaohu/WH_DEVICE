#include "StdAfx.h"
#include "CAccNotServiceStation.h"
#include "LogException.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

CAccNotServiceStation CAccNotServiceStation::theInstance;

//////////////////////////////////////////////////////////////////////////
/**
@brief      构造函数

@param      无

@retval     无

@exception  无
*/
//////////////////////////////////////////////////////////////////////////
CAccNotServiceStation::CAccNotServiceStation():CAccParamMaster()
{
	m_vec_NotServiceStationList_param.clear();	
}

//////////////////////////////////////////////////////////////////////////
/**
@brief      获取参数文件名

@param      无

@retval     无

@exception  无
*/
//////////////////////////////////////////////////////////////////////////
void CAccNotServiceStation::GetParamFileName()
{
	GetParamFileNameSub(ACC_PARAM_ID_0608);		// 获取参数文件名
}

//////////////////////////////////////////////////////////////////////////
/**
@brief      析构函数

@param      无

@retval     无

@exception  无
*/
//////////////////////////////////////////////////////////////////////////
CAccNotServiceStation::~CAccNotServiceStation(void)
{	
}

//////////////////////////////////////////////////////////////////////////
/**
@brief      读出当前文件的内容

@param      (i)LPBYTE lpData       文件内容

@retval     none

@exception  CSysException
*/
//////////////////////////////////////////////////////////////////////////
void CAccNotServiceStation::ParseBody(LPBYTE lpData)
{
	try{    
		// 偏移量设置
		m_vec_NotServiceStationList_param.clear();
		lpData += m_section[0].startPosition;
		if (m_section[0].recordCount != 0) { 			
			WORD notServiceList_len;
			memcpy(&notServiceList_len,lpData,2);
			lpData += 2;      
			for(int i = 0;i<notServiceList_len;i++){
				NO_SERV_STATION_CODE tmpserviceFeeInfo;
				//停运车站表
				tmpserviceFeeInfo.noServStationCode = BCD2int((char*)lpData, 2);
				lpData += 2;		
				m_vec_NotServiceStationList_param.push_back(tmpserviceFeeInfo);
			}
		}
	}
	// 异常捕获
	catch(CSysException&) {
		throw;
	}
	catch (...) {
		throw CLogException(CLogException::OTHER_ERR, _T(__FILE__), __LINE__);
	}
}

//////////////////////////////////////////////////////////////////////////
/**
@brief      读出分段信息

@param      (i)LPBYTE lpData       文件内容

@retval     none

@exception  CSysException
*/
//////////////////////////////////////////////////////////////////////////
void CAccNotServiceStation::ParseSectionInfo(LPBYTE lpData)
{
	try{
		if(m_packageHead.sectionCount != 0){			
			for(int i =0;i<m_packageHead.sectionCount;i++){
				m_section[i].startPosition = mltol(ComMakeLong(*lpData, *(lpData + 1), *(lpData + 2), *(lpData + 3)));
				lpData+=4;
				m_section[i].recordCount = mltol(ComMakeLong(*lpData, *(lpData + 1), *(lpData + 2), *(lpData + 3)));
				lpData+=4;
			}
		}
	}
	// 异常捕获
	catch(CSysException&) {
		throw;
	}
	catch (...) {
		throw CLogException(CLogException::OTHER_ERR, _T(__FILE__), __LINE__);
	}
}

//////////////////////////////////////////////////////////////////////////
/**
@brief      检查电文结构调整部分内容正确性

@param      (i)LPBYTE           电文信息

@retval     bool   true:正确  false：不正确

@exception  无
*/
//////////////////////////////////////////////////////////////////////////
bool CAccNotServiceStation::CheckBodyInfo(LPBYTE lpData)
{   
	return true;
}

//////////////////////////////////////////////////////////////////////////
/**
@brief      停运车站表参数

@param      none

@retval     vector<FARE_TYPE_LIST>   停运车站表参数向量数组

@exception  无
*/
//////////////////////////////////////////////////////////////////////////
const vector<CAccNotServiceStation::NO_SERV_STATION_CODE>&  CAccNotServiceStation::GetNotServiceStationListParam()
{
	return m_vec_NotServiceStationList_param;
}

//////////////////////////////////////////////////////////////////////////
/**
@brief       判定是否是停运车站

@param      (in)WORD StationCode     车站代码

@retval     bool  true:是停运车站     false:不是停运车站

@exception  无
*/
//////////////////////////////////////////////////////////////////////////
bool  CAccNotServiceStation::IsNotServiceStationParam(WORD StationCode)
{
	vector<CAccNotServiceStation::NO_SERV_STATION_CODE>::iterator  it;

	for (it=m_vec_NotServiceStationList_param.begin(); it!=m_vec_NotServiceStationList_param.end(); it++){
		if(it->noServStationCode == StationCode){
			return true;
		}
	}
	return false;
}



