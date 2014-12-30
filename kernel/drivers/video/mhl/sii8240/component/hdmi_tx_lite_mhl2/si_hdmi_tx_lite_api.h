/**********************************************************************************
 * Si8240 Linux Driver
 *
 * Copyright (C) 2011-2012 Silicon Image Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the
 * GNU General Public License for more details.
 *
 **********************************************************************************/

void SiiHdmiTxLiteInitialize (bool_t enableHdcp);
void SiiHdmiTxLiteHandleEvents (uint8_t HdcpStatus,uint8_t queryData);

void SiiHdmiTxLiteDisableEncryption (void);

void SiiHdmiTxLiteHpdStatusChange(bool_t connected);
void SiiHdmiTxLiteTmdsActive(void);
void SiiHdmiTxLiteTmdsInactive(void);

uint8_t SiiHdmiTxLiteReadEdid (void);
bool_t SiiHdmiTxLiteSinkIsHdmi (void);
void	AppNotifyMhlDownStreamHPDStatusChange(bool_t connected);

void AppDisableOutput (void);
void SiiHdmiTxLiteHdmiDrvTmdsActive(void);


void MhlTxLiteSetInfoFrame(InfoFrameType_e eventParam, void *pRefData);

void	SiiDrvVideoMute( void );
void	SiiDrvVideoUnmute( void );

