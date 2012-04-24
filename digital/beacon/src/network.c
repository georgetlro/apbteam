/* network.c */
/* Beacon network management. {{{
 *
 * Copyright (C) 2012 Florent Duchon
 *
 * APBTeam:
 *        Web: http://apbteam.org/
 *      Email: team AT apbteam DOT org
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * }}} */

#include <types.h>
#include <configServer.h>
#include <zdo.h>
#include "configuration.h"
#include "network.h"
#include "debug_avr.h"
#include "led.h"


// Endpoint parameters
static SimpleDescriptor_t simpleDescriptor = { APP_ENDPOINT, APP_PROFILE_ID, 1, 1, 0, 0 , NULL, 0, NULL };
static APS_RegisterEndpointReq_t endpointParams;
static ZDO_StartNetworkReq_t networkParams; // request params for ZDO_StartNetworkReq
APS_DataReq_t config;


// Network related variables
AppMessageBuffer_t zigbit_tx_buffer;
extern AppState_t appState;
extern DeviceType_t deviceType;


static uint8_t retryCounter = 0;                // Data sending retries counter
// Leave request, used for router to leave the network when communication was interrupted
static ZDO_ZdpReq_t leaveReq;


/* This function intializes network parameters */
void network_init(void)
{
	if(deviceType == DEVICE_TYPE_COORDINATOR)
	{
		bool rx_on_idle = true;
		CS_WriteParameter(CS_RX_ON_WHEN_IDLE_ID, &rx_on_idle);
	}
	if(deviceType == DEVICE_TYPE_END_DEVICE)
	{
		bool rx_on_idle = false;
		CS_WriteParameter(CS_RX_ON_WHEN_IDLE_ID, &rx_on_idle);		
	}
	
	/*False = random ID */
	/* True = static short address */
	bool unique_addr = true;
	CS_WriteParameter(CS_NWK_UNIQUE_ADDR_ID,&unique_addr);
	
  	uint16_t nwkAddr=CS_NWK_ADDR; 
  	CS_WriteParameter(CS_NWK_ADDR_ID, &nwkAddr);
	
	// Set the deviceType value to Config Server	
	CS_WriteParameter(CS_DEVICE_TYPE_ID, &deviceType);

	appState = APP_NETWORK_JOINING_STATE;
}


/* This function starts the network according to the defined configuraiton*/
void network_start(void)
{
	/* Activate the network status led blink */
	led_start_blink();
	
	networkParams.ZDO_StartNetworkConf = ZDO_StartNetworkConf;
	// start network
	ZDO_StartNetworkReq(&networkParams);
}

/* This function returns the network status */
uint16_t network_get_status(void)
{
	return appState;
}

/* ZDO_StartNetwork primitive confirmation callback */
void ZDO_StartNetworkConf(ZDO_StartNetworkConf_t* confirmInfo)
{
	if (confirmInfo->status == ZDO_SUCCESS_STATUS)
	{
		retryCounter = 0;
		
		appState = APP_NETWORK_JOINED_STATE;
		
		// Set application endpoint properties
		endpointParams.simpleDescriptor = &simpleDescriptor;
		endpointParams.APS_DataInd = APS_DataIndication;
		// Register endpoint
		APS_RegisterEndpointReq(&endpointParams);
		
		// Configure the message structure
		config.dstAddrMode = APS_SHORT_ADDRESS;					// Short addressing mode
#ifdef TYPE_COOR
		config.dstAddress.shortAddress = BROADCAST_ADDR_ALL;		// Destination address	
#else
		config.dstAddress.shortAddress = 0x0000;					// Destination address
#endif
		config.profileId = APP_PROFILE_ID;						// Profile ID
		config.dstEndpoint = APP_ENDPOINT;						// Desctination endpoint
		config.clusterId = APP_CLUSTER_ID;						// Desctination cluster ID
		config.srcEndpoint = APP_ENDPOINT;						// Source endpoint
		config.asdu = &zigbit_tx_buffer.message;							// application message pointer
		config.asduLength = 3 + sizeof(zigbit_tx_buffer.message.messageId);		// actual application message length
		config.txOptions.acknowledgedTransmission = 0;				// Acknowledged transmission enabled
		config.radius = 0;										// Use maximal possible radius
		config.APS_DataConf = APS_DataConf;						// Confirm handler    Z
		
		/* Stop the network status led blink */
		led_stop_blink();
	}
	else
	{
//  		uprintf("CONNECTION FAILED. confirmInfo->status = %x\n\r",confirmInfo->status);
	}
}



/* Update network status event handler */
void ZDO_MgmtNwkUpdateNotf(ZDO_MgmtNwkUpdateNotf_t *nwkParams)
{
	switch (nwkParams->status)
	{
		case ZDO_NETWORK_STARTED_STATUS:
			break;
		case ZDO_NETWORK_LOST_STATUS:
		{
			APS_UnregisterEndpointReq_t unregEndpoint;
			unregEndpoint.endpoint = endpointParams.simpleDescriptor->endpoint;
			APS_UnregisterEndpointReq(&unregEndpoint);
			// try to rejoin the network
			appState = APP_NETWORK_JOINING_STATE;
			break;
		}
		case ZDO_NWK_UPDATE_STATUS:
			break;
		default:
			break;
	}
}



/* brief Handler of aps data sent confirmation */
void APS_DataConf(APS_DataConf_t* confInfo)
{
	if (APS_SUCCESS_STATUS != confInfo->status)
	{
		retryCounter++;
		if (MAX_RETRIES_BEFORE_REJOIN == retryCounter)
		{
			network_leave();
		}
		else
		{
			// Data not delivered, resend.
// 			send_data();
		}
		return;
	}
	retryCounter = 0;
}


/* APS data indication handler */
void APS_DataIndication(APS_DataInd_t* indData)
{
	AppMessage_t *appMessage = (AppMessage_t *) indData->asdu;
	// Data received indication
	switch(appMessage->data[0])
	{
		case 0x42: // COMMANDE JACK
// 			jack = appMessage->data[2];
			break;
		case 0x43: // Update ANGLE
// 			beacon_number = appMessage->data[1];
// 			angle_received = appMessage->data[2];
// 			update_position(beacon_number,angle_received);
			break;
		default:
			uprintf("Unknown data type received = %x\r\n",appMessage->data[0]);
			break;
	}
}


/* Leave network */
void network_leave(void)
{
	ZDO_MgmtLeaveReq_t *zdpLeaveReq = &leaveReq.req.reqPayload.mgmtLeaveReq;
	APS_UnregisterEndpointReq_t unregEndpoint;

	appState = APP_NETWORK_LEAVING_STATE;

	unregEndpoint.endpoint = endpointParams.simpleDescriptor->endpoint;
	APS_UnregisterEndpointReq(&unregEndpoint);

	leaveReq.ZDO_ZdpResp =  zdpLeaveResp;
	leaveReq.reqCluster = MGMT_LEAVE_CLID;
	leaveReq.dstAddrMode = EXT_ADDR_MODE;
	leaveReq.dstExtAddr = 0;
	zdpLeaveReq->deviceAddr = 0;
	zdpLeaveReq->rejoin = 0;
	zdpLeaveReq->removeChildren = 1;
	zdpLeaveReq->reserved = 0;
	ZDO_ZdpReq(&leaveReq);
}
 

/* Leave network response */
void zdpLeaveResp(ZDO_ZdpResp_t *zdpResp)
{
	// Try to rejoin the network
	appState = APP_NETWORK_JOINING_STATE;

	(void)zdpResp;
}


void send_data(uint8_t type, uint8_t data)
{
	zigbit_tx_buffer.message.data[0]=type;
	zigbit_tx_buffer.message.data[1]=data;
	APS_DataReq(&config);
}

// void send_angle(int angle_degree)
// {
// 	zigbit_tx_buffer.message.data[0]=0x43;
// 	zigbit_tx_buffer.message.data[1]=beacon_id;
// 	zigbit_tx_buffer.message.data[2]=angle_degree;	
// }



/* Wakeup event handler (dummy) */
void ZDO_WakeUpInd(void)
{
}

/* Stub for ZDO Binding Indication */
void ZDO_BindIndication(ZDO_BindInd_t *bindInd) 
{
	(void)bindInd;
}

/* Stub for ZDO Unbinding Indication */
void ZDO_UnbindIndication(ZDO_UnbindInd_t *unbindInd)
{
	(void)unbindInd;
}

/* This function returns the LQI of the joined network */
uint8_t network_get_lqi(void)
{
	ZDO_GetLqiRssi_t lqiRssi;

	lqiRssi.nodeAddr = 0;
	ZDO_GetLqiRssi(&lqiRssi);
	
	return lqiRssi.lqi;
}

/* This function returns the RSSI of the joined network */
int8_t network_get_rssi(void)
{
	ZDO_GetLqiRssi_t lqiRssi;

	lqiRssi.nodeAddr = 0;
	ZDO_GetLqiRssi(&lqiRssi);
	
	return lqiRssi.rssi;
}


  