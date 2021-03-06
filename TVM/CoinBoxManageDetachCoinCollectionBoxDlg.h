#pragma once
#include "OperationDlg.h"

/**
@brief Ӳ�һ�����ж��
*/
class CCoinBoxManageDetachCoinCollectionBoxDlg : public COperationDlg
{
	DECLARE_DYNAMIC(CCoinBoxManageDetachCoinCollectionBoxDlg)
	DECLARE_MESSAGE_MAP()

public:
	CCoinBoxManageDetachCoinCollectionBoxDlg(CService* pService);
	~CCoinBoxManageDetachCoinCollectionBoxDlg();

	enum { IDD = IDD_20101_SYSTEM_SET_DLG };

	void InitDetachCoinCollectionBoxInfo();
	void ShowDetachCoinCollectionBoxInfo();					// ��ʾӲ�һ�����״̬
	void ShowDoDetachCoinCollectionBoxInfo();				// ��ʾӲ�һ�����ж��״̬
	void SetColor(int row,CString status);					// ����״̬��ʾ����ɫ
protected:
	virtual void UpdateUI();								// ������ʾ��ɻص�����

private:	
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnActivate(UINT nState, CWnd* pWndOther, BOOL bMinimized);

	LABEL_GROUP_SIX m_lableGroup[15];
};