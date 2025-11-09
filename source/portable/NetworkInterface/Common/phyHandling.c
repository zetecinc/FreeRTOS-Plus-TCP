/*
 * FreeRTOS+TCP <DEVELOPMENT BRANCH>
 * Copyright (C) 2022 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

// markh 7nov25: This is a stripped-down version for Leopard, supporting only the STM32H755 Nucleo board.
//               The idea is to have a clear foundation for eventual KSZ9893R implementation.


/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

#include "phyHandling.h"

/* As the following 3 macro's are OK in most situations, and so they're not
 * included in 'FreeRTOSIPConfigDefaults.h'.
 * Users can change their values in the project's 'FreeRTOSIPConfig.h'. */
#ifndef phyPHY_MAX_RESET_TIME_MS
    #define phyPHY_MAX_RESET_TIME_MS    1000U
#endif

#ifndef phyPHY_MAX_NEGOTIATE_TIME_MS
    #define phyPHY_MAX_NEGOTIATE_TIME_MS    3000U
#endif

#ifndef phySHORT_DELAY_MS
    #define phySHORT_DELAY_MS    50U
#endif

/* Naming and numbering of basic PHY registers. */
#define phyREG_00_BMCR             0x00U    /* Basic Mode Control Register. */
#define phyREG_01_BMSR             0x01U    /* Basic Mode Status Register. */
#define phyREG_02_PHYSID1          0x02U    /* PHYS ID 1 */
#define phyREG_03_PHYSID2          0x03U    /* PHYS ID 2 */
#define phyREG_04_ADVERTISE        0x04U    /* Advertisement control reg */

/* Naming and numbering of extended PHY registers. */
#define PHYREG_10_PHYSTS           0x10U    /* 16 PHY status register Offset */
#define phyREG_19_PHYCR            0x19U    /* 25 RW PHY Control Register */
#define phyREG_1F_PHYSPCS          0x1FU    /* 31 RW PHY Special Control Status */

/* Bit fields for 'phyREG_00_BMCR', the 'Basic Mode Control Register'. */
#define phyBMCR_FULL_DUPLEX        0x0100U  /* Full duplex. */
#define phyBMCR_AN_RESTART         0x0200U  /* Auto negotiation restart. */
#define phyBMCR_ISOLATE            0x0400U  /* 1 = Isolates 0 = Normal operation. */
#define phyBMCR_AN_ENABLE          0x1000U  /* Enable auto negotiation. */
#define phyBMCR_SPEED_100          0x2000U  /* Select 100Mbps. */
#define phyBMCR_RESET              0x8000U  /* Reset the PHY. */

/* Bit fields for 'phyREG_19_PHYCR', the 'PHY Control Register'. */
#define PHYCR_MDIX_EN              0x8000U  /* Enable Auto MDIX. */
#define PHYCR_MDIX_FORCE           0x4000U  /* Force MDIX crossed. */

#define phyBMSR_AN_COMPLETE        0x0020U  /* Auto-Negotiation process completed */

#define phyBMSR_LINK_STATUS        0x0004U

#define phyPHYSTS_LINK_STATUS      0x0001U  /* PHY Link mask */
#define phyPHYSTS_SPEED_STATUS     0x0002U  /* PHY Speed mask */
#define phyPHYSTS_DUPLEX_STATUS    0x0004U  /* PHY Duplex mask */

/* Bit fields for 'phyREG_1F_PHYSPCS
 *  001 = 10BASE-T half-duplex
 *  101 = 10BASE-T full-duplex
 *  010 = 100BASE-TX half-duplex
 *  110 = 100BASE-TX full-duplex
 */
#define phyPHYSPCS_SPEED_MASK      0x000CU
#define phyPHYSPCS_SPEED_10        0x0004U
#define phyPHYSPCS_FULL_DUPLEX     0x0010U

/*
 * Description of all capabilities that can be advertised to
 * the peer (usually a switch or router).
 */

#define phyADVERTISE_CSMA       0x0001U     /* Supports IEEE 802.3u: Fast Ethernet at 100 Mbit/s */
#define phyADVERTISE_10HALF     0x0020U     /* Try for 10mbps half-duplex. */
#define phyADVERTISE_10FULL     0x0040U     /* Try for 10mbps full-duplex. */
#define phyADVERTISE_100HALF    0x0080U     /* Try for 100mbps half-duplex. */
#define phyADVERTISE_100FULL    0x0100U     /* Try for 100mbps full-duplex. */

#define phyADVERTISE_ALL                            \
    ( phyADVERTISE_10HALF | phyADVERTISE_10FULL |   \
      phyADVERTISE_100HALF | phyADVERTISE_100FULL | \
      phyADVERTISE_CSMA )

/* Send a reset command to a set of PHY-ports. */
static uint32_t xPhyReset( EthernetPhy_t * pxPhyObject );

static BaseType_t xHas_1F_PHYSPCS( uint32_t ulPhyID )
{
    BaseType_t xResult = pdFALSE;

    switch( ulPhyID )
    {
        case PHY_ID_LAN8742A:
            xResult = pdTRUE;
            break;

        default:
            /* Has no 0x1F register "PHY Special Control Status". */
            break;
    }

    return xResult;
}


static BaseType_t xHas_19_PHYCR( uint32_t ulPhyID )
{
    BaseType_t xResult = pdFALSE;

    switch( ulPhyID )
    {
        case PHY_ID_LAN8742A:
            xResult = pdTRUE;
            break;

        default:
            break;
    }

    return xResult;
}


/* Initialise the struct and assign a PHY-read and -write function. */
void vPhyInitialise( EthernetPhy_t * pxPhyObject,
                     xApplicationPhyReadHook_t fnPhyRead,
                     xApplicationPhyWriteHook_t fnPhyWrite )
{
    memset( ( void * ) pxPhyObject, 0, sizeof( *pxPhyObject ) );

    pxPhyObject->fnPhyRead = fnPhyRead;
    pxPhyObject->fnPhyWrite = fnPhyWrite;
}


/* Discover all PHY's connected by polling 32 indexes ( zero-based ) */
BaseType_t xPhyDiscover( EthernetPhy_t * pxPhyObject )
{
    BaseType_t xPhyAddress = 0;  // markh: correct for single PHY (LAN8742A); for KSZ9893R we want either 1 or 2,
                                 // whichever is the external RJ45 (assuming we don't want to manage the instrument
                                 // PHY also as it's always connected!)

    uint32_t ulLowerID;
    pxPhyObject->fnPhyRead( xPhyAddress, phyREG_03_PHYSID2, &ulLowerID );

    /* A valid PHY id can not be all zeros or all ones. */
    if ( ulLowerID != ( uint16_t ) ~0U )   // markh 7nov25 based in forum discussion (virtual PHYs can have ID==0)
    {
        uint32_t ulUpperID;
        uint32_t ulPhyID;

        pxPhyObject->fnPhyRead( xPhyAddress, phyREG_02_PHYSID1, &ulUpperID );
        ulPhyID = ( ( ( uint32_t ) ulUpperID ) << 16 ) | ( ulLowerID & 0xFFF0U );

        pxPhyObject->ucPhyIndex = ( uint8_t ) xPhyAddress;
        pxPhyObject->ulPhyID = ulPhyID;

        FreeRTOS_printf( ( "PHY ID %X\n", ( unsigned int ) pxPhyObject->ulPhyID ) );
        return 1;
    }

    return 0;
}


/* Send a reset command to a set of PHY-ports. */
static uint32_t xPhyReset( EthernetPhy_t * pxPhyObject )
{
    uint32_t ulConfig;
    TickType_t xRemainingTime;
    TimeOut_t xTimer;

    uint32_t ulDone = 0;

    /* Set the RESET bit high. */
    BaseType_t xPhyAddress = pxPhyObject->ucPhyIndex;

    /* Read Control register. */
    pxPhyObject->fnPhyRead( xPhyAddress, phyREG_00_BMCR, &ulConfig );
    pxPhyObject->fnPhyWrite( xPhyAddress, phyREG_00_BMCR, ulConfig | phyBMCR_RESET );

    xRemainingTime = ( TickType_t ) pdMS_TO_TICKS( phyPHY_MAX_RESET_TIME_MS );
    vTaskSetTimeOutState( &xTimer );

    /* The reset should last less than a second. */
    for( ; ; )
    {
        pxPhyObject->fnPhyRead( xPhyAddress, phyREG_00_BMCR, &ulConfig );

        if( ( ulConfig & phyBMCR_RESET ) == 0 )
        {
            FreeRTOS_printf( ( "xPhyReset: phyBMCR_RESET ready\n" ) );
            ulDone = 1;
            break;
        }

        if( xTaskCheckForTimeOut( &xTimer, &xRemainingTime ) != pdFALSE )
        {
            FreeRTOS_printf( ( "xPhyReset: phyBMCR_RESET timed out\n" ) );
            break;
        }

        /* Block for a while */
        vTaskDelay( pdMS_TO_TICKS( phySHORT_DELAY_MS ) );
    }

    if( ulDone == 0 )
    {
        /* The reset operation timed out, clear the bit manually. */
        pxPhyObject->fnPhyRead( xPhyAddress, phyREG_00_BMCR, &ulConfig );
        pxPhyObject->fnPhyWrite( xPhyAddress, phyREG_00_BMCR, ulConfig & ~phyBMCR_RESET );
    }

    vTaskDelay( pdMS_TO_TICKS( phySHORT_DELAY_MS ) );

    return ulDone;
}


BaseType_t xPhyConfigure( EthernetPhy_t * pxPhyObject,
                          const PhyProperties_t * pxPhyProperties )
{
    uint32_t ulConfig, ulAdvertise;

    if( pxPhyObject->ulPhyID == 1 )
    {
        FreeRTOS_printf( ( "xPhyConfigure: No PHY's detected.\n" ) );
        return -1;
    }

    configASSERT( (pxPhyProperties->ucSpeed == ( uint8_t ) PHY_SPEED_AUTO)
                && (pxPhyProperties->ucDuplex == ( uint8_t ) PHY_DUPLEX_AUTO) )
    ulAdvertise = phyADVERTISE_ALL;

        
    /* Send a reset command to a set of PHY-ports. */
    xPhyReset( pxPhyObject );

    BaseType_t xPhyAddress = pxPhyObject->ucPhyIndex;
    uint32_t ulPhyID = pxPhyObject->ulPhyID;

    /* Write advertise register. */
    pxPhyObject->fnPhyWrite( xPhyAddress, phyREG_04_ADVERTISE, ulAdvertise );

    /*
      *      AN_EN        AN1         AN0       Forced Mode
      *        0           0           0        10BASE-T, Half-Duplex
      *        0           0           1        10BASE-T, Full-Duplex
      *        0           1           0        100BASE-TX, Half-Duplex
      *        0           1           1        100BASE-TX, Full-Duplex
      *      AN_EN        AN1         AN0       Advertised Mode
      *        1           0           0        10BASE-T, Half/Full-Duplex
      *        1           0           1        100BASE-TX, Half/Full-Duplex
      *        1           1           0        10BASE-T Half-Duplex
      *                                         100BASE-TX, Half-Duplex
      *        1           1           1        10BASE-T, Half/Full-Duplex
      *                                         100BASE-TX, Half/Full-Duplex
      */

    /* Read Control register. */
    pxPhyObject->fnPhyRead( xPhyAddress, phyREG_00_BMCR, &ulConfig );

    ulConfig |= (phyBMCR_AN_ENABLE | phyBMCR_SPEED_100 | phyBMCR_FULL_DUPLEX);

    if( xHas_19_PHYCR( ulPhyID ) )
    {
        uint32_t ulPhyControl;
        /* Read PHY Control register. */
        pxPhyObject->fnPhyRead( xPhyAddress, phyREG_19_PHYCR, &ulPhyControl );

        /* Clear bits which might get set: */
        ulPhyControl &= ~( PHYCR_MDIX_EN | PHYCR_MDIX_FORCE );

        if( pxPhyProperties->ucMDI_X == PHY_MDIX_AUTO )
        {
            ulPhyControl |= PHYCR_MDIX_EN;
        }
        else if( pxPhyProperties->ucMDI_X == PHY_MDIX_CROSSED )
        {
            /* Force direct link = Use crossed RJ45 cable. */
            ulPhyControl &= ~PHYCR_MDIX_FORCE;
        }
        else
        {
            /* Force crossed link = Use direct RJ45 cable. */
            ulPhyControl |= PHYCR_MDIX_FORCE;
        }

        /* update PHY Control Register. */
        pxPhyObject->fnPhyWrite( xPhyAddress, phyREG_19_PHYCR, ulPhyControl );
    }

    FreeRTOS_printf( ( "+TCP: advertise: %04X config %04X\n", ( unsigned int ) ulAdvertise, ( unsigned int ) ulConfig ) );
    
    /* Keep these values for later use. */
    pxPhyObject->ulBCRValue = ulConfig & ~phyBMCR_ISOLATE;
    pxPhyObject->ulACRValue = ulAdvertise;

    return 0;
}


/* xPhyStartAutoNegotiation() is the alternative xPhyFixedValue():
 * It sets the BMCR_AN_RESTART bit and waits for the auto-negotiation completion
 * ( phyBMSR_AN_COMPLETE ). */
BaseType_t xPhyStartAutoNegotiation( EthernetPhy_t * pxPhyObject )
{
    uint32_t ulDone = 0;
    uint32_t ulRegValue;
    TickType_t xRemainingTime;
    TimeOut_t xTimer;

    BaseType_t xPhyAddress = pxPhyObject->ucPhyIndex;

    /* Enable Auto-Negotiation. */
    pxPhyObject->fnPhyWrite( xPhyAddress, phyREG_04_ADVERTISE, pxPhyObject->ulACRValue );
    pxPhyObject->fnPhyWrite( xPhyAddress, phyREG_00_BMCR, pxPhyObject->ulBCRValue | phyBMCR_AN_RESTART );

    xRemainingTime = ( TickType_t ) pdMS_TO_TICKS( phyPHY_MAX_NEGOTIATE_TIME_MS );
    vTaskSetTimeOutState( &xTimer );

    /* Wait until the auto-negotiation will be completed */
    for( ; ; )
    {
        if( ulDone == 0 )
        {
            pxPhyObject->fnPhyRead( xPhyAddress, phyREG_01_BMSR, &ulRegValue );

            if( ( ulRegValue & phyBMSR_AN_COMPLETE ) != 0 )
            {
                ulDone = 1;
                break;
            }
        }

        if( xTaskCheckForTimeOut( &xTimer, &xRemainingTime ) != pdFALSE )
        {
            FreeRTOS_printf( ( "xPhyStartAutoNegotiation: phyBMSR_AN_COMPLETE timed out\n" ) );
            break;
        }

        vTaskDelay( pdMS_TO_TICKS( phySHORT_DELAY_MS ) );
    }

    if( ulDone == 1 )
    {
        pxPhyObject->ulLinkStatus = 0;

        //uint32_t ulPhyID = pxPhyObject->ulPhyID;

        /* Clear the 'phyBMCR_AN_RESTART'  bit. */
        pxPhyObject->fnPhyWrite( xPhyAddress, phyREG_00_BMCR, pxPhyObject->ulBCRValue );
        pxPhyObject->fnPhyRead( xPhyAddress, phyREG_01_BMSR, &ulRegValue );

        if( ( ulRegValue & phyBMSR_LINK_STATUS ) != 0U )
        {
            pxPhyObject->ulLinkStatus = 1;
        }

//        if( xHas_1F_PHYSPCS( ulPhyID ) )  // markh: must be true (LAN8742A)
//        {
            /* 31 RW PHY Special Control Status */
            uint32_t ulControlStatus;

            pxPhyObject->fnPhyRead( xPhyAddress, phyREG_1F_PHYSPCS, &ulControlStatus );
            ulRegValue = 0;

            if( ( ulControlStatus & phyPHYSPCS_FULL_DUPLEX ) != 0 )
            {
                ulRegValue |= phyPHYSTS_DUPLEX_STATUS;
            }

            if( ( ulControlStatus & phyPHYSPCS_SPEED_MASK ) == phyPHYSPCS_SPEED_10 )
            {
                ulRegValue |= phyPHYSTS_SPEED_STATUS;
            }
//        }

        FreeRTOS_printf( ( "Autonego ready: %08x: %s duplex %u mbit %s status\n",
                            ( unsigned int ) ulRegValue,
                            ( ulRegValue & phyPHYSTS_DUPLEX_STATUS ) ? "full" : "half",
                            ( ulRegValue & phyPHYSTS_SPEED_STATUS ) ? 10 : 100,
                            ( pxPhyObject->ulLinkStatus != 0 ) ? "high" : "low" ) );

        if( ( ulRegValue & phyPHYSTS_DUPLEX_STATUS ) != ( uint32_t ) 0U )
        {
            pxPhyObject->xPhyProperties.ucDuplex = PHY_DUPLEX_FULL;
        }
        else
        {
            pxPhyObject->xPhyProperties.ucDuplex = PHY_DUPLEX_HALF;
        }

        if( ( ulRegValue & phyPHYSTS_SPEED_STATUS ) != 0 )
        {
            pxPhyObject->xPhyProperties.ucSpeed = PHY_SPEED_10;
        }
        else
        {
            pxPhyObject->xPhyProperties.ucSpeed = PHY_SPEED_100;
        }
    }

    return 0;
}
/*-----------------------------------------------------------*/

BaseType_t xPhyCheckLinkStatus( EthernetPhy_t * pxPhyObject,
                                BaseType_t xHadReception )
{
    uint32_t ulStatus;
    BaseType_t xNeedCheck = pdFALSE;

    if( xHadReception > 0 )
    {
        /* A packet was received. No need to check for the PHY status now,
         * but set a timer to check it later on. */
        vTaskSetTimeOutState( &( pxPhyObject->xLinkStatusTimer ) );
        pxPhyObject->xLinkStatusRemaining = pdMS_TO_TICKS( ipconfigPHY_LS_HIGH_CHECK_TIME_MS );

        if( pxPhyObject->ulLinkStatus == 0 )
        {
            pxPhyObject->ulLinkStatus = 1;
            FreeRTOS_printf( ( "xPhyCheckLinkStatus: PHY LS now %s\n", pxPhyObject->ulLinkStatus != 0 ? "high" : "low" ) );
            xNeedCheck = pdTRUE;
        }
    }
    else if( xTaskCheckForTimeOut( &( pxPhyObject->xLinkStatusTimer ), &( pxPhyObject->xLinkStatusRemaining ) ) != pdFALSE )
    {
        /* Frequent checking the PHY Link Status can affect for the performance of Ethernet controller.
         * As long as packets are received, no polling is needed.
         * Otherwise, polling will be done when the 'xLinkStatusTimer' expires. */
        BaseType_t xPhyAddress = pxPhyObject->ucPhyIndex;
        if( pxPhyObject->fnPhyRead( xPhyAddress, phyREG_01_BMSR, &ulStatus ) == 0 )
        {
            if( ( pxPhyObject->ulLinkStatus != 0 ) != !!( ulStatus & phyBMSR_LINK_STATUS ) )
            {
                if( ( ulStatus & phyBMSR_LINK_STATUS ) != 0 )
                {
                    pxPhyObject->ulLinkStatus = 1;
                }
                else
                {
                    pxPhyObject->ulLinkStatus = 0;
                }

                FreeRTOS_printf( ( "xPhyCheckLinkStatus: PHY LS now %s", pxPhyObject->ulLinkStatus != 0 ? "high" : "low" ) );
                xNeedCheck = pdTRUE;
            }
        }

        vTaskSetTimeOutState( &( pxPhyObject->xLinkStatusTimer ) );

        if( pxPhyObject->ulLinkStatus != 0 )
        {
            /* The link status is high, so don't poll the PHY too often. */
            pxPhyObject->xLinkStatusRemaining = pdMS_TO_TICKS( ipconfigPHY_LS_HIGH_CHECK_TIME_MS );
        }
        else
        {
            /* The link status is low, polling may be done more frequently. */
            pxPhyObject->xLinkStatusRemaining = pdMS_TO_TICKS( ipconfigPHY_LS_LOW_CHECK_TIME_MS );
        }
    }

    return xNeedCheck;
}
