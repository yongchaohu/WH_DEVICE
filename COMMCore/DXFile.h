#pragma once
#include "CoreExp.h"
#include "Util.h"
#include "Func.h"

class CORE_API CDXFile
{
public:
//////////////////////////////////////////////////////////////////////////
/*
@brief   构造函数   

@param

@retval     

@exception  
*/
//////////////////////////////////////////////////////////////////////////
	CDXFile(CString fileName,CString backupFileName);

//////////////////////////////////////////////////////////////////////////
/*
@brief   析构函数   

@param

@retval     

@exception  
*/
//////////////////////////////////////////////////////////////////////////
	virtual ~CDXFile();


	typedef enum _err_detaitl{
		FILE_OPERATION_OK   = 0,        // 文件操作成功
		FILE_READ_FAIL	    = 102, 	    // 文件读取异常
		FILE_OPEN_FAIL	    = 104, 	    // 文件打开异常
		FILE_RESEURE_FAIL_BACKUP   = 201,		// 文件二重化失败
		FILE_RESEURE_FAIL_MAIN     = 202,		// 文件二重化主文件失败
	} ERR_DETAITL;
//////////////////////////////////////////////////////////////////////////
/*
@brief  读二取制文件    

@param

@retval     

@exception  
*/
//////////////////////////////////////////////////////////////////////////
	ERR_DETAITL WriteBinFile(const char* lpData, const ULONG nLen,bool hasSum = false);
//////////////////////////////////////////////////////////////////////////
/*
@brief 写二进制文件

@param

@retval     

@exception  
*/
//////////////////////////////////////////////////////////////////////////
	ERR_DETAITL ReadBinFile(VARIABLE_DATA& buffer,bool hasSum = false);
	ERR_DETAITL ReadBinFile(VARIABLE_DATA& buffer, Func<bool(VARIABLE_DATA&)>* validator/* = NULL*/, bool hasSum/* = false*/);

//////////////////////////////////////////////////////////////////////////
/*
@brief      清空文件

@param

@retval     

@exception  
*/
//////////////////////////////////////////////////////////////////////////
	ERR_DETAITL ClearFile();

//////////////////////////////////////////////////////////////////////////
/*
@brief      二重化文件

@param

@retval     

@exception  
*/
//////////////////////////////////////////////////////////////////////////
	static bool ResuceFile(CString mainFilePath,CString backupFilePath,Func<bool(CString)>* funcValidator);

//////////////////////////////////////////////////////////////////////////
/*
@brief     二重化文件 

@param

@retval     

@exception  
*/
//////////////////////////////////////////////////////////////////////////
	static bool ResuceFile(CString mainFilePath,CString backupFilePath,bool hasSum = false);

//////////////////////////////////////////////////////////////////////////
/*
@brief   删除二重化文件   

@param

@retval     

@exception  
*/
//////////////////////////////////////////////////////////////////////////
	static bool DeleteResuceFile(CString mainFilePath,CString backupFilePath);
//////////////////////////////////////////////////////////////////////////
/*
@brief   检查并创建目录   

@param

@retval     

@exception  
*/
//////////////////////////////////////////////////////////////////////////
	static bool CheckAndCreateDirectory(CString mainFolderPath,CString backupFolderPath);
//////////////////////////////////////////////////////////////////////////
/*
@brief  重命名二重化文件    

@param

@retval     

@exception  
*/
//////////////////////////////////////////////////////////////////////////
	static bool RenameResuceFile(CString mainFilePath,CString backupFilePath,CString mainNewFilePath,CString backupNewFilePath);

	//////////////////////////////////////////////////////////////////////////
	/*
	@brief      验证带sum文件文件

	@param

	@retval     

	@exception  
	*/
	//////////////////////////////////////////////////////////////////////////
	static bool ValidateFileWithSum(CString fileName);
	//////////////////////////////////////////////////////////////////////////
	/*
	@brief  验证文件存在    

	@param

	@retval     

	@exception  
	*/
	//////////////////////////////////////////////////////////////////////////
	static bool ValidateFileExist(CString fileName);

private:
	CString m_mainFileName;				// 基本文件名
	CString m_backupFileName;	


};