CREATE TABLE IF NOT EXISTS tb_Transaction(   
	iTransID	INTEGER		PRIMARY KEY AUTOINCREMENT,
	iUDSN		TEXT,
	dtTransTime	DATETIME	DEFAULT(0),
	iUdType		INTEGER		DEFAULT(0),
	iUdSubtype	INTEGER		DEFAULT(0),
	iCardType	INTEGER		DEFAULT(0),
	iProductType	INTEGER		DEFAULT(0),
	iProductSubType	INTEGER		DEFAULT(0),
	iOrigin		INTEGER		DEFAULT(0),
	iDestination	INTEGER		DEFAULT(0),
	DepositAmount	INTEGER		DEFAULT(0),
	TaxAmount	INTEGER		DEFAULT(0),
	CardFeeAmount	INTEGER		DEFAULT(0),
	iUnitAmount	INTEGER		DEFAULT(0),
	iQuantity	INTEGER		DEFAULT(0),
	iPaymentMethod	INTEGER		DEFAULT(0),
	iTotalAmount	INTEGER		DEFAULT(0),
	CoinInsertAmount	INTEGER	DEFAULT(0),
	CoinState	INTEGER		DEFAULT(0),
	BankNoteAcceptAmount	INTEGER	DEFAULT(0),
	BanknoteAcceptState	INTEGER	DEFAULT(0),
	iChangeAmount	INTEGER		DEFAULT(0),
	iCoinChangeAmount	INTEGER	DEFAULT(0),
	iCoinChangeState	INTEGER	DEFAULT(0),
	iBanknoteChangeAmount	INTEGER	DEFAULT(0),
	iBanknoteChangeState	INTEGER	DEFAULT(0),
	iUnReleaseedMediaQuantity	INTEGER		DEFAULT(0),
	iPaymentState	INTEGER		DEFAULT(0),
	iPayCardNo	TEXT,
	iPrintState	INTEGER		DEFAULT(0),
	iState		INTEGER		DEFAULT(0),
	txtReserve	TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_CoinInsertRecord(	
	iTransID	INTEGER,
	dtOperationTime	DATETIME	DEFAULT(0),
	iCoinTypeCode	INTEGER		DEFAULT(0),
	iQuantity	INTEGER		DEFAULT(0),
	txtReserve	TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_CoinChangeRecord(	
	iTransID	INTEGER,
	dtOperationTime	DATETIME	DEFAULT(0),
	txtCoinBoxID  TEXT,
	iCoinTypeCode	INTEGER		DEFAULT(0),
	iQuantity	INTEGER		DEFAULT(0),
	txtReserve	TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_CoinAcceptRecord(	
	iTransID	INTEGER,
	dtOperationTime	DATETIME	DEFAULT(0),
	txtCoinTempID  TEXT,
	txtAcceptCoinBoxID	TEXT,
	iCoinTypeCode	INTEGER		DEFAULT(0),
	iQuantity	INTEGER		DEFAULT(0),
	txtReserve	TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_BanknoteInsertRecord(	
	iTransID	INTEGER,
	dtOperationTime	DATETIME	DEFAULT(0),
	iCoinTypeCode	INTEGER		DEFAULT(0),
	iQuantity	INTEGER		DEFAULT(0),
	txtReserve	TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_BanknoteChangeRecord(	
	iTransID	INTEGER,
	dtOperationTime	DATETIME	DEFAULT(0),
	txtCoinBoxID	TEXT,
	iCoinTypeCode	INTEGER		DEFAULT(0),
	iQuantity	INTEGER		DEFAULT(0),
	txtReserve	TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_BanknoteAcceptRecord(	
	iTransID	INTEGER,
	dtOperationTime	DATETIME	DEFAULT(0),
	txtAcceptCoinBoxID	TEXT,
	iCoinTypeCode	INTEGER		DEFAULT(0),
	iQuantity	INTEGER		DEFAULT(0),
	txtReserve	TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_BanknoteDiscardRecord(	
	iTransID	INTEGER,
	dtOperationTime	DATETIME	DEFAULT(0),
	txtCoinBoxID	TEXT,
	iCoinTypeCode	INTEGER		DEFAULT(0),
	iQuantity	INTEGER		DEFAULT(0),
	txtReserve	TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_TicketOperationRecord(
	iTransID		INTEGER,
	dtOperationTime		DATETIME	DEFAULT(0),
	iMediaType		INTEGER,
	txtTicketBoxID	TEXT,
	iCardSerialNumber	TEXT		DEFAULT(NULL),
	iEncodeState		INTEGER		DEFAULT(0),
	iReleaseState		INTEGER		DEFAULT(0),
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_ExceptionRecord(
	iTransID		INTEGER,
	dtOperationTime		DATETIME	DEFAULT(0),
	iExceptionType	INTEGER,
	IExceptionAmount		INTEGER		DEFAULT(0),
	txtReserve		TEXT		DEFAULT(NULL)
);


CREATE TABLE IF NOT EXISTS tb_CoinAddOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtOperationTime		DATETIME	DEFAULT(0),
	txtCoinAddBoxID	TEXT	DEFAULT(0),
	txtCoinChangeBoxID	TEXT,
	iCoinTypeCode		INTEGER	DEFAULT(0),
	iQuantityBeforeAdd		INTEGER,
	iQuantityAdd		INTEGER,
	iQuantityAfterAdd		INTEGER,
	txtOperatorID	TEXT,
	txtComment	TEXT		DEFAULT(NULL),
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_CoinChangeBoxOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtOperationTime		DATETIME	DEFAULT(0),
	iOperationType		INTEGER,
	txtCoinChangeBoxID	TEXT,
	iCoinTypeCode		INTEGER	DEFAULT(0),
	iQuantity			INTEGER,
	txtOperatorID		TEXT,
	txtComment			TEXT		DEFAULT(NULL),
	txtReserve			TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_CoinCollectOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtOperationTime		DATETIME	DEFAULT(0),
	txtCoinChangeBoxID	TEXT	DEFAULT(0),
	iCoinTypeCode		INTEGER	DEFAULT(0),
	iQuantityBeforeCollect		INTEGER,
	iQuantityCollect		INTEGER,
	iQuantityAfterCollect		INTEGER,
	txtCoinCollectBoxID	TEXT,
	txtOperatorID	TEXT,
	txtComment	TEXT		DEFAULT(NULL),
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_CoinCollectBoxRemoveOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtTransTime		DATETIME	DEFAULT(0),
	txtCoinCollectBoxID     TEXT	DEFAULT(0),
	iTotalAmount		INTEGER	DEFAULT(0),
	txtOperatorID	TEXT,
	txtComment	TEXT		DEFAULT(NULL),
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_CoinBoxCashDetail(
	iID		INTEGER,
	iCoinTypeCode		INTEGER	DEFAULT(0),
	iQuantity		INTEGER,
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_BanknoteOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtTransTime			DATETIME	DEFAULT(0),
	iOperationType		INTEGER,
	txtBanknoteBoxID	TEXT	DEFAULT(0),
	iBanknoteTypeCode	INTEGER	DEFAULT(0),
	iQuantity			INTEGER,
	txtOperatorID		TEXT,
	txtComment			TEXT		DEFAULT(NULL),
	txtReserve			TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_BanknoteBoxInstallOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtTransTime		DATETIME	DEFAULT(0),
	txtBanknoteBoxID	TEXT	DEFAULT(0),
	iTotalAmount		INTEGER	DEFAULT(0),
	txtOperatorID	TEXT,
	txtComment	TEXT		DEFAULT(NULL),
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_BanknoteBoxRemoveOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtTransTime		DATETIME	DEFAULT(0),
	txtBanknoteBoxID	TEXT	DEFAULT(0),
	iTotalAmount		INTEGER	DEFAULT(0),
	txtOperatorID	TEXT,
	txtComment	TEXT		DEFAULT(NULL),
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_BanknoteBoxCashDetail(
	iID		INTEGER,
	iBanknoteTypeCode		INTEGER	DEFAULT(0),
	iQuantity		INTEGER,
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_TokenBoxInstallOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtOperationTime		DATETIME	DEFAULT(0),
	txtTokenBoxID	TEXT	DEFAULT(0),
	iQuantity		INTEGER	DEFAULT(0),
	txtOperatorID	TEXT,
	txtComment	TEXT		DEFAULT(NULL),
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_TokenBoxRemoveOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtOperationTime		DATETIME	DEFAULT(0),
	txtTokenBoxID	TEXT	DEFAULT(0),
	iOriginalQuantity		INTEGER	DEFAULT(0),
	iRemoveQuantity		INTEGER	DEFAULT(0),
	txtOperatorID	TEXT,
	txtComment	TEXT		DEFAULT(NULL),
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_TokenBoxAddOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtOperationTime		DATETIME	DEFAULT(0),
	txtTokenAddBoxID	TEXT	DEFAULT(0),
	txtTokenHopperBoxID	TEXT	DEFAULT(0),
	iQuantityBeforeAdd		INTEGER	DEFAULT(0),
	iQuantityAdd		INTEGER	DEFAULT(0),
	iQuantityAfterAdd		INTEGER	DEFAULT(0),
	txtOperatorID	TEXT,
	txtComment	TEXT		DEFAULT(NULL),
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_TokenCollectOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtOperationTime		DATETIME	DEFAULT(0),
	txtTokenBoxID	TEXT	DEFAULT(0),
	iQuantityBeforeCollect		INTEGER	DEFAULT(0),
	iQuantityCollect		INTEGER	DEFAULT(0),
	iQuantityAfterCollect		INTEGER	DEFAULT(0),
	txtTokenCollectBoxID	TEXT	DEFAULT(0),
	txtOperatorID	TEXT,
	txtComment	TEXT		DEFAULT(NULL),
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_TokenCollectBoxRemoveOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtTransTime		DATETIME	DEFAULT(0),
	txtTokenBoxID	TEXT	DEFAULT(0),
	iQuantity		INTEGER	DEFAULT(0),
	txtOperatorID	TEXT,
	txtComment	TEXT		DEFAULT(NULL),
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_CardBoxInstallOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtOperationTime		DATETIME	DEFAULT(0),
	txtCardBoxID	TEXT	DEFAULT(0),
	iQuantity		INTEGER	DEFAULT(0),
	txtOperatorID	TEXT,
	txtComment	TEXT		DEFAULT(NULL),
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_CardBoxRemoveOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtOperationTime		DATETIME	DEFAULT(0),
	txtCardBoxID	TEXT	DEFAULT(0),
	iOriginalQuantity		INTEGER	DEFAULT(0),
	iRemoveQuantity		INTEGER	DEFAULT(0),
	txtOperatorID	TEXT,
	txtComment	TEXT		DEFAULT(NULL),
	txtReserve		TEXT		DEFAULT(NULL)
);

CREATE TABLE IF NOT EXISTS tb_CardCollectBoxRemoveOperationRecord(
	iID		INTEGER		PRIMARY KEY AUTOINCREMENT,
	dtTransTime		DATETIME	DEFAULT(0),
	txtCardBoxID	TEXT	DEFAULT(0),
	iQuantity		INTEGER	DEFAULT(0),
	txtOperatorID	TEXT,
	txtComment	TEXT		DEFAULT(NULL),
	txtReserve		TEXT		DEFAULT(NULL)
);


/*
CREATE INDEX IF NOT EXISTS IDX_tb_Task_txtTaskName ON tb_Task (txtTaskName);

CREATE INDEX IF NOT EXISTS IDX_tb_Template_txtTemplateName ON tb_Template(txtTemplateName);

CREATE INDEX IF NOT EXISTS IDX_tb_Log_Ticket_Process_dtTime ON tb_Log_Ticket_Process (dtTime);

CREATE INDEX IF NOT EXISTS IDX_tb_Log_Task_Execute_txtTaskID ON tb_Log_Task_Execute (txtTaskID);

CREATE INDEX IF NOT EXISTS IDX_tb_Log_Task_Execute_dtTime ON tb_Log_Task_Execute (dtTime);

CREATE INDEX IF NOT EXISTS IDX_tb_Log_Ticket_Exception_dtTime ON tb_Log_Ticket_Exception (dtTime);

CREATE INDEX IF NOT EXISTS IDX_tb_Log_Magazine_Operation_dtTime ON tb_Log_Magazine_Operation (dtTime);

CREATE INDEX IF NOT EXISTS IDX_tb_Log_Device_Exception_dtTime ON tb_Log_Device_Exception (dtTime);

CREATE INDEX IF NOT EXISTS IDX_tb_Log_Device_Operation_dtTime ON tb_Log_Device_Operation (dtTime);

CREATE INDEX IF NOT EXISTS IDX_tb_Log_Personalize_Process_dtTime ON tb_Log_Personalize_Process (dtTime);

CREATE INDEX IF NOT EXISTS IDX_tb_BlankCard_Info_txtSupplyTime_iSupplyBatchNo ON tb_BlankCard_Info(txtSupplyTime,iSupplyBatchNo);

CREATE INDEX IF NOT EXISTS IDX_tb_Person_Info_txtName ON tb_Person_Info (txtName);

CREATE INDEX IF NOT EXISTS IDX_tb_Person_Info_dtTime ON tb_Person_Info (dtTime);


INSERT INTO tb_Template(
	txtTemplateId,
	txtTemplateName,
	txtReserve)
	values('0000000011001','PersonalizeFixedTemplate',''
);

INSERT INTO tb_Template_Personalize(
	txtTemplateId,txtAreaName,iPersionProperty,txtFont,txtColor,iVerticalCoordinate,	
	iHorizontalCoordinate,iWidth,iHeight,txtPicPath)
	values('0000000011001','StaffName','2','Arial,36','FFFFFFFF',
		'470','50','570','80',''
);

INSERT INTO tb_Template_Personalize(
	txtTemplateId,txtAreaName,iPersionProperty,txtFont,txtColor,iVerticalCoordinate,	
	iHorizontalCoordinate,iWidth,iHeight,txtPicPath)
	values('0000000011001','PapersType','4','Arial,36','FFFFFFFF',
		'190','50','570','80',''
);
INSERT INTO tb_Template_Personalize(
	txtTemplateId,txtAreaName,iPersionProperty,txtFont,txtColor,iVerticalCoordinate,	
	iHorizontalCoordinate,iWidth,iHeight,txtPicPath)
	values('0000000011001','PerpersID','5','Arial,36','FFFFFFFF',
		'330','50','570','80',''
);

INSERT INTO tb_Template_Personalize(
	txtTemplateId,txtAreaName,iPersionProperty,txtFont,txtColor,iVerticalCoordinate,	
	iHorizontalCoordinate,iWidth,iHeight,txtPicPath)
	values('0000000011001','FrontPicPath','7','','',
		'0','0','1016','648','C:\\ES\\card background\\staff ticket - Front.bmp'
);
INSERT INTO tb_Template_Personalize(
	txtTemplateId,txtAreaName,iPersionProperty,txtFont,txtColor,iVerticalCoordinate,	
	iHorizontalCoordinate,iWidth,iHeight,txtPicPath)
	values('0000000011001','BackPicPath','8','','',
		'0','0','1016','648','C:\\ES\\card background\\staff ticket - Back.bmp'
);

INSERT INTO tb_Template_Personalize(
	txtTemplateId,txtAreaName,iPersionProperty,txtFont,txtColor,iVerticalCoordinate,	
	iHorizontalCoordinate,iWidth,iHeight,txtPicPath)
	values('0000000011001','PhotoPath','9','','',
		'120','671','295','413','D:\\ES_Data\\picture\\photo\\StaffPhoto.bmp'
);
*/