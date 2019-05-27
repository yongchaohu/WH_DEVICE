#pragma once;
#include "stdafx.h"
#include "DataCommand.h"

namespace Data
{
	//////////////////////////////////////////////////////////////////////////
	/*
	@brief      不带返回数据的数据库操作命令

	*/
	//////////////////////////////////////////////////////////////////////////
	class TVMDB_API CNonQueryCommand:public CDataCommand
	{
	public:

		//////////////////////////////////////////////////////////////////////////
		/*
		@brief  构造函数    

		@param CString& sql 执行的sql语句。

		@retval     

		@exception  
		*/
		//////////////////////////////////////////////////////////////////////////
		CNonQueryCommand(CString& sql);

		//////////////////////////////////////////////////////////////////////////
		/*
		@brief    析造函数  

		@param

		@retval     

		@exception  
		*/
		//////////////////////////////////////////////////////////////////////////
		virtual ~CNonQueryCommand();

		//////////////////////////////////////////////////////////////////////////
		/*
		@brief    取得影响行数 

		@param

		@retval     

		@exception  
		*/
		//////////////////////////////////////////////////////////////////////////
		int GetAffectRows();

	protected:

		//////////////////////////////////////////////////////////////////////////
		/*
		@brief    执行命令函数，该函数中执行SQL语句  

		@param

		@retval     

		@exception  
		*/
		//////////////////////////////////////////////////////////////////////////
		virtual long ExecuteCommand();
	private:
		
		//////////////////////////////////////////////////////////////////////////
		/*
		@brief  执行的sql语句  
		*/
		//////////////////////////////////////////////////////////////////////////
		CHAR* sqlStatement;

		//////////////////////////////////////////////////////////////////////////
		/*
		@brief  影响行数  
		*/
		//////////////////////////////////////////////////////////////////////////
		int affectRows;
	};
};