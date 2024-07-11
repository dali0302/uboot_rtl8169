/*
0612_v4: 
	base on v2022.01 uboot driver, porting inbox driver 8125b component into the uboot driver to support RTL8125B
	
0612_v5: 
	1.upgrading uboot base code v2022.01 to v2024.04
	2.keep CONFIG_DM_ETH branch judgement for improving compatibility
	3.According to the coding style of the open source community, the uboot driver is also written in the style of the inbox driver(eg. latest v6.9.5 r8169_main.c) and no longer refers to the rtk driver.
	4.if there is a conflict between the two(uboot v2024.04, and inbox v6.9.5) in terms of code style, please use the uboot driver as the standard, because of uboot source code structor is diff from kernel driver.

0612_v6/7:
	1.correct some compile error
*/

// SPDX-License-Identifier: GPL-2.0+
/*
 * rtl8169.c : U-Boot driver for the RealTek RTL8169
 *
 * Masami Komiya (mkomiya@sonare.it)
 *
 * Most part is taken from r8169.c of etherboot
 *
 */

/**************************************************************************
*    r8169.c: Etherboot device driver for the RealTek RTL-8169 Gigabit
*    Written 2003 by Timothy Legge <tlegge@rogers.com>
*
*    Portions of this code based on:
*	r8169.c: A RealTek RTL-8169 Gigabit Ethernet driver
*		for Linux kernel 2.4.x.
*
*    Written 2002 ShuChen <shuchen@realtek.com.tw>
*	  See Linux Driver for full information
*
*    Linux Driver Version 1.27a, 10.02.2002
*
*    Thanks to:
*	Jean Chen of RealTek Semiconductor Corp. for
*	providing the evaluation NIC used to develop
*	this driver.  RealTek's support for Etherboot
*	is appreciated.
*
*    REVISION HISTORY:
*    ================
*
*    v1.0	11-26-2003	timlegge	Initial port of Linux driver
*    v1.5	01-17-2004	timlegge	Initial driver output cleanup
*
*    Indent Options: indent -kr -i8
***************************************************************************/
/*
 * 26 August 2006 Mihai Georgian <u-boot@linuxnotincluded.org.uk>
 * Modified to use le32_to_cpu and cpu_to_le32 properly
 */
#include <common.h>
#include <cpu_func.h>
#include <dm.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>
#include <memalign.h>
#include <net.h>

/*
dali: the latest uboot rtl8169.c default enable CONFIG_DM_ETH，however some customer's platform still use the older version of the uboot. so I keep the ifdef/ifndef to improve compatibility
*/
#ifndef CONFIG_DM_ETH
#include <netdev.h>
#endif

#include <asm/cache.h>
#include <asm/io.h>
#include <pci.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/mii.h>

//dali: default disable debug macro
#undef DEBUG_RTL8169
#undef DEBUG_RTL8169_TX
#undef DEBUG_RTL8169_RX

/*
#ifndef DEBUG_RTL8169
#define DEBUG_RTL8169 1
#endif

#ifndef DEBUG_RTL8169_TX
#define DEBUG_RTL8169_TX 1
#endif

#ifndef DEBUG_RTL8169_RX
#define DEBUG_RTL8169_RX 1
#endif
*/

/*
dali: As mentioned above, provide compatibility with some old platforms
*/
//#undef RK_v201709
#define RK_v201709 1

#undef NEEDS_WrRamCodeToMicroP

#define drv_version "v1.5"
#define drv_date "01-17-2004"

static unsigned long ioaddr;

/* Condensed operations for readability. */
#define currticks()	get_timer(0)

/* media options */
#define MAX_UNITS 8
static int media[MAX_UNITS] = { -1, -1, -1, -1, -1, -1, -1, -1 };

/* MAC address length*/
#define MAC_ADDR_LEN	6

/* max supported gigabit ethernet frame size -- must be at least (dev->mtu+14+4).*/
#define MAX_ETH_FRAME_SIZE	1536

#ifndef ETH_ZLEN
#define ETH_ZLEN				60
#endif

#define TX_FIFO_THRESH 256	/* In bytes */

#define RX_FIFO_THRESH	7	/* 7 means NO threshold, Rx buffer level before first PCI xfer.	 */
#define RX_DMA_BURST	6	/* Maximum PCI burst, '6' is 1024 */
#define TX_DMA_BURST	6	/* Maximum PCI burst, '6' is 1024 */
#define EarlyTxThld	0x3F	/* 0x3F means NO early transmit */
#define RxPacketMaxSize 0x0800	/* Maximum size supported is 16K-1 */
#define InterFrameGap	0x03	/* 3 means InterFrameGap = the shortest one */

#define NUM_TX_DESC	1	/* Number of Tx descriptor registers */
#ifdef CONFIG_SYS_RX_ETH_BUFFER
  #define NUM_RX_DESC	CONFIG_SYS_RX_ETH_BUFFER
#else
  #define NUM_RX_DESC	4	/* Number of Rx descriptor registers */
#endif
#define RX_BUF_SIZE	1536	/* Rx Buffer size */
#define RX_BUF_LEN	8192

#define RTL_MIN_IO_SIZE 0x80
#define TX_TIMEOUT  (6*HZ)

/* write/read MMIO register. Notice: {read,write}[wl] do the necessary swapping */
#define RTL_W8(reg, val8)	writeb((val8), (void *)(ioaddr + (reg)))
#define RTL_W16(reg, val16)	writew((val16), (void *)(ioaddr + (reg)))
#define RTL_W32(reg, val32)	writel((val32), (void *)(ioaddr + (reg)))
#define RTL_R8(reg)		readb((void *)(ioaddr + (reg)))
#define RTL_R16(reg)		readw((void *)(ioaddr + (reg)))
#define RTL_R32(reg)		readl((void *)(ioaddr + (reg)))

#define bus_to_phys(a)	pci_mem_to_phys((pci_dev_t)(unsigned long)dev->priv, \
	(pci_addr_t)(unsigned long)a)
#define phys_to_bus(a)	pci_phys_to_mem((pci_dev_t)(unsigned long)dev->priv, \
	(phys_addr_t)a)

//dali: rtk v9.013.02
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE  1
#endif

#ifndef false
#define false 0
#endif

#ifndef true
#define true  1
#endif

//dali: rtk v9.013.02
enum bits {
        BIT_0 = (1 << 0),
        BIT_1 = (1 << 1),
        BIT_2 = (1 << 2),
        BIT_3 = (1 << 3),
        BIT_4 = (1 << 4),
        BIT_5 = (1 << 5),
        BIT_6 = (1 << 6),
        BIT_7 = (1 << 7),
        BIT_8 = (1 << 8),
        BIT_9 = (1 << 9),
        BIT_10 = (1 << 10),
        BIT_11 = (1 << 11),
        BIT_12 = (1 << 12),
        BIT_13 = (1 << 13),
        BIT_14 = (1 << 14),
        BIT_15 = (1 << 15),
        BIT_16 = (1 << 16),
        BIT_17 = (1 << 17),
        BIT_18 = (1 << 18),
        BIT_19 = (1 << 19),
        BIT_20 = (1 << 20),
        BIT_21 = (1 << 21),
        BIT_22 = (1 << 22),
        BIT_23 = (1 << 23),
        BIT_24 = (1 << 24),
        BIT_25 = (1 << 25),
        BIT_26 = (1 << 26),
        BIT_27 = (1 << 27),
        BIT_28 = (1 << 28),
        BIT_29 = (1 << 29),
        BIT_30 = (1 << 30),
        BIT_31 = (1 << 31)
};

enum RTL8169_registers {
	MAC0 = 0,		/* Ethernet hardware address. */
	MAR0 = 8,		/* Multicast filter. */
	TxDescStartAddrLow = 0x20,
	TxDescStartAddrHigh = 0x24,
	TxHDescStartAddrLow = 0x28,
	TxHDescStartAddrHigh = 0x2c,
	FLASH = 0x30,
	ERSR = 0x36,
	ChipCmd = 0x37,
	TxPoll_8169 = 0x38,
	IntrMask_8169 = 0x3C,
	IntrStatus_8169 = 0x3E,
	TxConfig = 0x40,
	RxConfig = 0x44,
	RxMissed = 0x4C,
	Cfg9346 = 0x50,
	Config0 = 0x51,
	Config1 = 0x52,
	Config2 = 0x53,
	Config3 = 0x54,
	Config4 = 0x55,
	Config5 = 0x56,
	MultiIntr = 0x5C,
	PHYAR = 0x60,
	TBICSR = 0x64,
	TBI_ANAR = 0x68,
	TBI_LPAR = 0x6A,
	PHYstatus = 0x6C,
	PMCH = 0x6F,
	EPHYAR = 0x80,
	OCPDR = 0xB0,
	PHYOCP = 0xB8,
	RxMaxSize = 0xDA,
	CPlusCmd = 0xE0,
	RxDescStartAddrLow = 0xE4,
	RxDescStartAddrHigh = 0xE8,
	EarlyTxThres = 0xEC,
	FuncEvent = 0xF0,
	FuncEventMask = 0xF4,
	FuncPresetState = 0xF8,
	FuncForceEvent = 0xFC,
};

enum RTL8125_registers {
	INT_CFG0_8125 = 0x34,
	IntrMask_8125 = 0x38,
	IntrStatus_8125 = 0x3C,
	INT_CFG1_8125 = 0x7a,
	TxPoll_8125 = 0x90,
	MACOCP = 0xB0,
};

enum RTL8169_register_content {
	/*InterruptStatusBits */
	SYSErr = 0x8000,
	PCSTimeout = 0x4000,
	SWInt = 0x0100,
	TxDescUnavail = 0x80,
	RxFIFOOver = 0x40,
	RxUnderrun = 0x20,
	RxOverflow = 0x10,
	TxErr = 0x08,
	TxOK = 0x04,
	RxErr = 0x02,
	RxOK = 0x01,

	/*RxStatusDesc */
	RxRES = 0x00200000,
	RxCRC = 0x00080000,
	RxRUNT = 0x00100000,
	RxRWT = 0x00400000,

	/*ChipCmdBits */
	CmdReset = 0x10,
	CmdRxEnb = 0x08,
	CmdTxEnb = 0x04,
	RxBufEmpty = 0x01,

	/*Cfg9346Bits */
	Cfg9346_Lock = 0x00,
	Cfg9346_Unlock = 0xC0,

	/*rx_mode_bits */
	AcceptErr = 0x20,
	AcceptRunt = 0x10,
	AcceptBroadcast = 0x08,
	AcceptMulticast = 0x04,
	AcceptMyPhys = 0x02,
	AcceptAllPhys = 0x01,

	/*RxConfigBits */
	RxCfgFIFOShift = 13,
	RxCfgDMAShift = 8,

	/*TxConfigBits */
	TxInterFrameGapShift = 24,
	TxDMAShift = 8,		/* DMA burst value (0-7) is shift this many bits */

	/*rtl8169_PHYstatus */
	TBI_Enable = 0x80,
	TxFlowCtrl = 0x40,
	RxFlowCtrl = 0x20,
	_1000bpsF = 0x10,
	_100bps = 0x08,
	_10bps = 0x04,
	LinkStatus = 0x02,
	FullDup = 0x01,

	/*GIGABIT_PHY_registers */
	PHY_CTRL_REG = 0,
	PHY_STAT_REG = 1,
	PHY_AUTO_NEGO_REG = 4,
	PHY_1000_CTRL_REG = 9,

	/*GIGABIT_PHY_REG_BIT */
	PHY_Restart_Auto_Nego = 0x0200,
	PHY_Enable_Auto_Nego = 0x1000,

	/* PHY_STAT_REG = 1; */
	PHY_Auto_Nego_Comp = 0x0020,

	/* PHY_AUTO_NEGO_REG = 4; */
	PHY_Cap_10_Half = 0x0020,
	PHY_Cap_10_Full = 0x0040,
	PHY_Cap_100_Half = 0x0080,
	PHY_Cap_100_Full = 0x0100,

	/* PHY_1000_CTRL_REG = 9; */
	PHY_Cap_1000_Full = 0x0200,

	PHY_Cap_Null = 0x0,

	/*_MediaType*/
	_10_Half = 0x01,
	_10_Full = 0x02,
	_100_Half = 0x04,
	_100_Full = 0x08,
	_1000_Full = 0x10,

	/*_TBICSRBit*/
	TBILinkOK = 0x02000000,

	//dali: rtk v9.013.02
	/* OCP GPHY access */
	OCPDR_Write = 0x80000000,
	OCPDR_Read = 0x00000000,
	OCPDR_Reg_Mask = 0xFF,
	OCPDR_Data_Mask = 0xFFFF,
	OCPDR_GPHY_Reg_shift = 16,
	OCPAR_Flag = 0x80000000,
	OCPAR_GPHY_Write = 0x8000F060,
	OCPAR_GPHY_Read = 0x0000F060,
	OCPR_Write = 0x80000000,
	OCPR_Read = 0x00000000,
	OCPR_Addr_Reg_shift = 16,
	OCPR_Flag = 0x80000000,
	OCP_STD_PHY_BASE_PAGE = 0x0A40,

	/* EPHY access */
	EPHYAR_Flag = 0x80000000,
	EPHYAR_Write = 0x80000000,
	EPHYAR_Read = 0x00000000,
	//dali: EPHYAR_Reg_Mask == 0x3f no matter in r8168 or r8125. but inbox driver set it to 0x1f. We decided to be consistent with the realtek driver.
	EPHYAR_Reg_Mask = 0x3f,
	EPHYAR_Reg_Mask_v2 = 0x7f,
	EPHYAR_Reg_shift = 16,
	EPHYAR_Data_Mask = 0xffff,

	//dali: inbox driver v6.9.5
	/* Config2 register p. 25 */
	ClkReqEn	= (1 << 7),	/* Clock Request Enable */
	MSIEnable	= (1 << 5),	/* 8169 only. Reserved in the 8168. */
	PCI_Clock_66MHz = 0x01,
	PCI_Clock_33MHz = 0x00,

	/* Config3 register p.25 */
	MagicPacket	= (1 << 5),	/* Wake up when receives a Magic Packet */
	LinkUp		= (1 << 4),	/* Wake up when the cable connection is re-established */
	Jumbo_En0	= (1 << 2),	/* 8168 only. Reserved in the 8168b */
	Rdy_to_L23	= (1 << 1),	/* L23 Enable */
	Beacon_en	= (1 << 0),	/* 8168 only. Reserved in the 8168b */

	/* Config5 register p.27 */
	BWF		= (1 << 6),	/* Accept Broadcast wakeup frame */
	MWF		= (1 << 5),	/* Accept Multicast wakeup frame */
	UWF		= (1 << 4),	/* Accept Unicast wakeup frame */
	Spi_en		= (1 << 3),
	LanWake		= (1 << 1),	/* LanWake enable/disable */
	PMEStatus	= (1 << 0),	/* PME status can be reset by PCI RST# */
	ASPM_en		= (1 << 0),	/* ASPM enable */
	
	/* FuncEvent/Misc */
	RxDv_Gated_En = 0x80000,
};

//Channel Wait Count
#define R8125_CHANNEL_WAIT_COUNT (20000)
#define R8125_CHANNEL_WAIT_TIME (1)  // 1us
#define R8125_CHANNEL_EXIT_DELAY_TIME (20)  //20us

static struct {
	const char *name;
	u8 version;		/* depend on RTL8169 docs */
	u32 RxConfigMask;	/* should clear the bits supported by this chip */
} rtl_chip_info[] = {
	{"RTL-8169", 0x00, 0xff7e1880,},
	{"RTL-8169", 0x04, 0xff7e1880,},
	{"RTL-8169", 0x00, 0xff7e1880,},
	{"RTL-8169s/8110s",	0x02, 0xff7e1880,},
	{"RTL-8169s/8110s",	0x04, 0xff7e1880,},
	{"RTL-8169sb/8110sb",	0x10, 0xff7e1880,},
	{"RTL-8169sc/8110sc",	0x18, 0xff7e1880,},
	{"RTL-8168b/8111sb",	0x30, 0xff7e1880,},
	{"RTL-8168b/8111sb",	0x38, 0xff7e1880,},
	{"RTL-8168c/8111c",	0x3c, 0xff7e1880,},
	{"RTL-8168d/8111d",	0x28, 0xff7e1880,},
	{"RTL-8168evl/8111evl",	0x2e, 0xff7e1880,},
	{"RTL-8168/8111g",	0x4c, 0xff7e1880,},
	{"RTL-8101e",		0x34, 0xff7e1880,},
	{"RTL-8100e",		0x32, 0xff7e1880,},
	{"RTL-8168h/8111h",	0x54, 0xff7e1880,},
	/*dali: compare to uboot v2024.04, follow the r8125_n.c setting about rx_mode. bit[14] added to enable RxSingleFetch additionally.*/
	{"RTL-8125B",		0x64, 0xff7e5880,},
};

enum _DescStatusBit {
	OWNbit = 0x80000000,
	EORbit = 0x40000000,
	FSbit = 0x20000000,
	LSbit = 0x10000000,
};

struct TxDesc {
	u32 status;
	u32 vlan_tag;
	u32 buf_addr;
	u32 buf_Haddr;
};

struct RxDesc {
	u32 status;
	u32 vlan_tag;
	u32 buf_addr;
	u32 buf_Haddr;
};

static unsigned char rxdata[RX_BUF_LEN];

#define RTL8169_DESC_SIZE 16

#if ARCH_DMA_MINALIGN > 256
#  define RTL8169_ALIGN ARCH_DMA_MINALIGN
#else
#  define RTL8169_ALIGN 256
#endif

/*
 * Warn if the cache-line size is larger than the descriptor size. In such
 * cases the driver will likely fail because the CPU needs to flush the cache
 * when requeuing RX buffers, therefore descriptors written by the hardware
 * may be discarded.
 *
 * This can be fixed by defining CONFIG_SYS_NONCACHED_MEMORY which will cause
 * the driver to allocate descriptors from a pool of non-cached memory.
 *
 * Hardware maintain D-cache coherency in RISC-V architecture.
 */
#if RTL8169_DESC_SIZE < ARCH_DMA_MINALIGN
#if !defined(CONFIG_SYS_NONCACHED_MEMORY) && \
	!CONFIG_IS_ENABLED(SYS_DCACHE_OFF) && !defined(CONFIG_X86) && !defined(CONFIG_RISCV)
#warning cache-line size is larger than descriptor size
#endif
#endif

/*
 * Create a static buffer of size RX_BUF_SZ for each TX Descriptor. All
 * descriptors point to a part of this buffer.
 */
DEFINE_ALIGN_BUFFER(u8, txb, NUM_TX_DESC * RX_BUF_SIZE, RTL8169_ALIGN);

/*
 * Create a static buffer of size RX_BUF_SZ for each RX Descriptor. All
 * descriptors point to a part of this buffer.
 */
DEFINE_ALIGN_BUFFER(u8, rxb, NUM_RX_DESC * RX_BUF_SIZE, RTL8169_ALIGN);

struct rtl8169_private {
	ulong iobase;
	void *mmio_addr;	/* memory map physical address */
	int chipset;
	unsigned long cur_rx;	/* Index into the Rx descriptor buffer of next Rx pkt. */
	unsigned long cur_tx;	/* Index into the Tx descriptor buffer of next Rx pkt. */
	unsigned long dirty_tx;
	struct TxDesc *TxDescArray;	/* Index of 256-alignment Tx Descriptor buffer */
	struct RxDesc *RxDescArray;	/* Index of 256-alignment Rx Descriptor buffer */
	unsigned char *RxBufferRings;	/* Index of Rx Buffer  */
	unsigned char *RxBufferRing[NUM_RX_DESC];	/* Index of Rx Buffer array */
	unsigned char *Tx_skbuff[NUM_TX_DESC];
	u16 cur_page;
	u8	HwHasWrRamCodeToMicroP;
	u16 sw_ram_code_ver;
	u16 hw_ram_code_ver;
} tpx;

static struct rtl8169_private *tpc;

static const unsigned int rtl8169_rx_config =
    (RX_FIFO_THRESH << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift);

static struct pci_device_id supported[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8125) },
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8161) },
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8167) },
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8168) },
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8169) },
	{}
};

static inline u16 map_phy_ocp_addr(u16 PageNum, u8 RegNum)
{
        u16 OcpPageNum = 0;
        u8 OcpRegNum = 0;
        u16 OcpPhyAddress = 0;

        if ( PageNum == 0 ) {
                OcpPageNum = OCP_STD_PHY_BASE_PAGE + ( RegNum / 8 );
                OcpRegNum = 0x10 + ( RegNum % 8 );
        } else {
                OcpPageNum = PageNum;
                OcpRegNum = RegNum;
        }

        OcpPageNum <<= 4;

        if ( OcpRegNum < 16 ) {
                OcpPhyAddress = 0;
        } else {
                OcpRegNum -= 16;
                OcpRegNum <<= 1;

                OcpPhyAddress = OcpPageNum + OcpRegNum;
        }


        return OcpPhyAddress;
}

static u32 mdio_real_direct_read_phy_ocp(u16 RegAddr)
{
        u32 data32;
        int i, value = 0;

        WARN_ON_ONCE(RegAddr % 2);
        data32 = RegAddr/2;
        data32 <<= OCPR_Addr_Reg_shift;

        RTL_W32(PHYOCP, data32);
        for (i = 0; i < 100; i++) {
                udelay(1);

                if (RTL_R32(PHYOCP) & OCPR_Flag)
                        break;
        }
        value = RTL_R32(PHYOCP) & OCPDR_Data_Mask;

        return value;
}

static u32 rtl8125_mdio_real_read_phy_ocp(u16 PageNum, u32 RegAddr)
{
        u16 ocp_addr;

        ocp_addr = map_phy_ocp_addr(PageNum, RegAddr);

        return mdio_real_direct_read_phy_ocp(ocp_addr);
}

static u32 mdio_real_read(u16 RegAddr)
{
        return rtl8125_mdio_real_read_phy_ocp(tpc->cur_page, RegAddr);
}

u32 rtl8125_mdio_read(u16 RegAddr)
{
        return mdio_real_read(RegAddr);
}

static void mdio_real_direct_write_phy_ocp(u16 RegAddr, u16 value)
{
        u32 data32;
        int i;

        WARN_ON_ONCE(RegAddr % 2);
        data32 = RegAddr/2;
        data32 <<= OCPR_Addr_Reg_shift;
        data32 |= OCPR_Write | value;

        RTL_W32(PHYOCP, data32);
        for (i = 0; i < 100; i++) {
                udelay(1);

                if (!(RTL_R32(PHYOCP) & OCPR_Flag))
                        break;
        }
}

static void rtl8125_mdio_real_write_phy_ocp(u16 PageNum, u32 RegAddr, u32 value)
{
        u16 ocp_addr;

        ocp_addr = map_phy_ocp_addr(PageNum, RegAddr);

        mdio_real_direct_write_phy_ocp(ocp_addr, value);
}

static void mdio_real_write(u16 RegAddr, u16 value)
{
        if (RegAddr == 0x1F) {
                tpc->cur_page = value;
                return;
        }
        rtl8125_mdio_real_write_phy_ocp(tpc->cur_page, RegAddr, value);
}

void rtl8125_mdio_write(u16 RegAddr, u16 value)
{
        mdio_real_write(RegAddr, value);
}

static u32 mdio_direct_read_phy_ocp(u16 RegAddr)
{
        return mdio_real_direct_read_phy_ocp(RegAddr);
}

static void mdio_direct_write_phy_ocp(u16 RegAddr, u16 value)
{
        mdio_real_direct_write_phy_ocp(RegAddr, value);
}

static void ClearAndSetEthPhyOcpBit(u16 addr, u16 clearmask, u16 setmask)
{
        u16 PhyRegValue;

        PhyRegValue = mdio_direct_read_phy_ocp(addr);
        PhyRegValue &= ~clearmask;
        PhyRegValue |= setmask;
        mdio_direct_write_phy_ocp(addr, PhyRegValue);
}

void SetEthPhyOcpBit(u16 addr, u16 mask)
{
        ClearAndSetEthPhyOcpBit(addr, 0, mask);
}

void ClearEthPhyOcpBit(u16 addr, u16 mask)
{
        ClearAndSetEthPhyOcpBit(addr, mask, 0);
}

void mdio_write(int RegAddr, int value)
{
	int i;

	if(tpc->chipset == 16){
		if(tpc->cur_page == 0x0)
			mdio_real_write(0x1f, 0x0000);
		else
			mdio_real_write(0x1f, tpc->cur_page);
		mdio_real_write(RegAddr, value);
	}
	else{
		RTL_W32(PHYAR, 0x80000000 | (RegAddr & 0xFF) << 16 | value);
		udelay(1000);

		for (i = 2000; i > 0; i--) {
			/* Check if the RTL8169 has completed writing to the specified MII register */
			if (!(RTL_R32(PHYAR) & 0x80000000)) {
				break;
			} else {
				udelay(100);
			}
		}
	}
}

int mdio_read(int RegAddr)
{
	int i, value = -1;
	if(tpc->chipset == 16){
		if(tpc->cur_page == 0x0)
			mdio_real_write(0x1f, 0x0000);
		else
			mdio_real_write(0x1f, tpc->cur_page);
		mdio_real_read(RegAddr);
		return  0;
	}
	else{
		RTL_W32(PHYAR, 0x0 | (RegAddr & 0xFF) << 16);
		udelay(1000);

		for (i = 2000; i > 0; i--) {
			/* Check if the RTL8169 has completed retrieving data from the specified MII register */
			if (RTL_R32(PHYAR) & 0x80000000) {
				value = (int) (RTL_R32(PHYAR) & 0xFFFF);
				break;
			} else {
				udelay(100);
			}
		}
		return value;
	}
}

/*dali: Temporarily remove
static void rtl8169_enable_cfg9346_write(void)
{
		RTL_W8(Cfg9346, RTL_R8(Cfg9346) | Cfg9346_Unlock);
}

static void rtl8169_disable_cfg9346_write(void)
{
		RTL_W8(Cfg9346, RTL_R8(Cfg9346) & ~Cfg9346_Unlock);
}
*/

void rtl8125_mac_ocp_write(u16 reg_addr, u16 value)
{
        u32 data32;

        WARN_ON_ONCE(reg_addr % 2);
        data32 = reg_addr/2;
        data32 <<= OCPR_Addr_Reg_shift;
        data32 += value;
        data32 |= OCPR_Write;

        RTL_W32(MACOCP, data32);
}

u32 rtl8125_mac_ocp_read(u16 reg_addr)
{
        u32 data32;
        u16 data16 = 0;

        WARN_ON_ONCE(reg_addr % 2);
        data32 = reg_addr/2;
        data32 <<= OCPR_Addr_Reg_shift;

        RTL_W32(MACOCP, data32);
        data16 = (u16)RTL_R32(MACOCP);

        return data16;
}

static void ClearAndSetMcuAccessRegBit(u16 addr, u16 clearmask, u16 setmask)
{
        u16 PhyRegValue;

        PhyRegValue = rtl8125_mac_ocp_read(addr);
        PhyRegValue &= ~clearmask;
        PhyRegValue |= setmask;
        rtl8125_mac_ocp_write(addr, PhyRegValue);
}

static void ClearMcuAccessRegBit(u16 addr, u16 mask)
{
        ClearAndSetMcuAccessRegBit(addr, mask, 0);
}

static int rtl8169_init_board(unsigned long dev_iobase, const char *name)
{
	int i;
	u32 tmp;

#ifdef DEBUG_RTL8169
	printf ("%s\n", __FUNCTION__);
#endif
	ioaddr = dev_iobase;

	/* Soft reset the chip. */
	RTL_W8(ChipCmd, CmdReset);

	/* Check that the chip has finished the reset. */
	for (i = 1000; i > 0; i--)
		if ((RTL_R8(ChipCmd) & CmdReset) == 0)
			break;
		else
			udelay(10);

	/* identify chip attached to board */
	tmp = RTL_R32(TxConfig);
	tmp = ((tmp & 0x7c000000) + ((tmp & 0x00800000) << 2)) >> 24;

	for (i = ARRAY_SIZE(rtl_chip_info) - 1; i >= 0; i--){
		if (tmp == rtl_chip_info[i].version) {
			tpc->chipset = i;
			#ifdef DEBUG_RTL8169
				printf("rtl8169_init_board tpc->chipset=%d, chipname:%s \n", tpc->chipset, rtl_chip_info[i].name);
			#endif
			goto match;
		}
	}

	/* if unknown chip, assume array element #0, original RTL-8169 in this case */
	printf("PCI device %s: unknown chip version, assuming RTL-8169\n",
	       name);
	printf("PCI device: TxConfig = 0x%lX\n", (unsigned long) RTL_R32(TxConfig));
	tpc->chipset = 0;

match:
#ifdef NEEDS_WrRamCodeToMicroP
	tpc->sw_ram_code_ver = 0x0b99;
#else
	tpc->sw_ram_code_ver = 0;
#endif
	return 0;
}

/*
 * TX and RX descriptors are 16 bytes. This causes problems with the cache
 * maintenance on CPUs where the cache-line size exceeds the size of these
 * descriptors. What will happen is that when the driver receives a packet
 * it will be immediately requeued for the hardware to reuse. The CPU will
 * therefore need to flush the cache-line containing the descriptor, which
 * will cause all other descriptors in the same cache-line to be flushed
 * along with it. If one of those descriptors had been written to by the
 * device those changes (and the associated packet) will be lost.
 *
 * To work around this, we make use of non-cached memory if available. If
 * descriptors are mapped uncached there's no need to manually flush them
 * or invalidate them.
 *
 * Note that this only applies to descriptors. The packet data buffers do
 * not have the same constraints since they are 1536 bytes large, so they
 * are unlikely to share cache-lines.
 */
static void *rtl_alloc_descs(unsigned int num)
{
	size_t size = num * RTL8169_DESC_SIZE;

#ifdef CONFIG_SYS_NONCACHED_MEMORY
	return (void *)noncached_alloc(size, RTL8169_ALIGN);
#else
	return memalign(RTL8169_ALIGN, size);
#endif
}

/*
 * Cache maintenance functions. These are simple wrappers around the more
 * general purpose flush_cache() and invalidate_dcache_range() functions.
 */

static void rtl_inval_rx_desc(struct RxDesc *desc)
{
#ifndef CONFIG_SYS_NONCACHED_MEMORY
	unsigned long start = (unsigned long)desc & ~(ARCH_DMA_MINALIGN - 1);
	unsigned long end = ALIGN(start + sizeof(*desc), ARCH_DMA_MINALIGN);

	invalidate_dcache_range(start, end);
#endif
}

static void rtl_flush_rx_desc(struct RxDesc *desc)
{
#ifndef CONFIG_SYS_NONCACHED_MEMORY
	flush_cache((unsigned long)desc, sizeof(*desc));
#endif
}

static void rtl_inval_tx_desc(struct TxDesc *desc)
{
#ifndef CONFIG_SYS_NONCACHED_MEMORY
	unsigned long start = (unsigned long)desc & ~(ARCH_DMA_MINALIGN - 1);
	unsigned long end = ALIGN(start + sizeof(*desc), ARCH_DMA_MINALIGN);

	invalidate_dcache_range(start, end);
#endif
}

static void rtl_flush_tx_desc(struct TxDesc *desc)
{
#ifndef CONFIG_SYS_NONCACHED_MEMORY
	flush_cache((unsigned long)desc, sizeof(*desc));
#endif
}

static void rtl_inval_buffer(void *buf, size_t size)
{
	unsigned long start = (unsigned long)buf & ~(ARCH_DMA_MINALIGN - 1);
	unsigned long end = ALIGN(start + size, ARCH_DMA_MINALIGN);

	invalidate_dcache_range(start, end);
}

static void rtl_flush_buffer(void *buf, size_t size)
{
	flush_cache((unsigned long)buf, size);
}

/**************************************************************************
RECV - Receive a frame
***************************************************************************/
#ifdef CONFIG_DM_ETH
static int rtl_recv_common(struct udevice *dev, unsigned long dev_iobase,
			   uchar **packetp)
#else
static int rtl_recv_common(pci_dev_t dev, unsigned long dev_iobase,
			   uchar **packetp)
#endif
{
	/* return true if there's an ethernet packet ready to read */
	/* nic->packet should contain data on return */
	/* nic->packetlen should contain length of data */
	#ifdef RK_v201709
		struct pci_child_platdata *pplat = dev_get_parent_platdata(dev);
	#else
		struct pci_child_plat *pplat = dev_get_parent_plat(dev);
	#endif									 
	int cur_rx;
	int length = 0;

#ifdef DEBUG_RTL8169_RX
	printf ("%s\n", __FUNCTION__);
#endif
	ioaddr = dev_iobase;

	cur_rx = tpc->cur_rx;

	rtl_inval_rx_desc(&tpc->RxDescArray[cur_rx]);

	if ((le32_to_cpu(tpc->RxDescArray[cur_rx].status) & OWNbit) == 0) {
		if (!(le32_to_cpu(tpc->RxDescArray[cur_rx].status) & RxRES)) {
			length = (int) (le32_to_cpu(tpc->RxDescArray[cur_rx].
						status) & 0x00001FFF) - 4;

			rtl_inval_buffer(tpc->RxBufferRing[cur_rx], length);
			memcpy(rxdata, tpc->RxBufferRing[cur_rx], length);

			if (cur_rx == NUM_RX_DESC - 1)
				tpc->RxDescArray[cur_rx].status =
					cpu_to_le32((OWNbit | EORbit) + RX_BUF_SIZE);
			else
				tpc->RxDescArray[cur_rx].status =
					cpu_to_le32(OWNbit + RX_BUF_SIZE);
#ifdef CONFIG_DM_ETH
			tpc->RxDescArray[cur_rx].buf_addr = cpu_to_le32(
				dm_pci_mem_to_phys(dev,
					(pci_addr_t)(unsigned long)
					tpc->RxBufferRing[cur_rx]));
#else
			tpc->RxDescArray[cur_rx].buf_addr = cpu_to_le32(
				pci_mem_to_phys(dev, (pci_addr_t)(unsigned long)
				tpc->RxBufferRing[cur_rx]));
#endif
			rtl_flush_rx_desc(&tpc->RxDescArray[cur_rx]);
#ifdef CONFIG_DM_ETH
			*packetp = rxdata;
#else
			net_process_received_packet(rxdata, length);
#endif
		} else {
			puts("Error Rx");
			length = -EIO;
		}
		cur_rx = (cur_rx + 1) % NUM_RX_DESC;
		tpc->cur_rx = cur_rx;
		return length;

	} else {
		u32 IntrStatus = IntrStatus_8169;
		
		if (pplat->device == 0x8125)
			IntrStatus = IntrStatus_8125;
		ushort sts = RTL_R8(IntrStatus);
		RTL_W8(IntrStatus, sts & ~(TxErr | RxErr | SYSErr));
		udelay(100);	/* wait */
	}
	tpc->cur_rx = cur_rx;
	return (0);		/* initially as this is called to flush the input */
}

#ifdef CONFIG_DM_ETH
int rtl8169_eth_recv(struct udevice *dev, int flags, uchar **packetp)
{
	struct rtl8169_private *priv = dev_get_priv(dev);

	return rtl_recv_common(dev, priv->iobase, packetp);
}
#else
static int rtl_recv(struct eth_device *dev)
{
	return rtl_recv_common((pci_dev_t)(unsigned long)dev->priv,
			       dev->iobase, NULL);
}
#endif /* nCONFIG_DM_ETH */

#define HZ 1000
/**************************************************************************
SEND - Transmit a frame
***************************************************************************/
#ifdef CONFIG_DM_ETH
static int rtl_send_common(struct udevice *dev, unsigned long dev_iobase,
			   void *packet, int length)
#else
static int rtl_send_common(pci_dev_t dev, unsigned long dev_iobase,
			   void *packet, int length)
#endif /* nCONFIG_DM_ETH */
{
	/* send the packet to destination */

	#ifdef RK_v201709
		struct pci_child_platdata *pplat = dev_get_parent_platdata(dev);
	#else
		struct pci_child_plat *pplat = dev_get_parent_plat(dev);
	#endif
	u32 to;
	u8 *ptxb;
	int entry = tpc->cur_tx % NUM_TX_DESC;
	u32 len = length;
	int ret;

#ifdef DEBUG_RTL8169_TX
	int stime = currticks();
	printf ("%s\n", __FUNCTION__);
	printf("sending %d bytes\n", len);
#endif

	ioaddr = dev_iobase;

	/* point to the current txb incase multiple tx_rings are used */
	ptxb = tpc->Tx_skbuff[entry * MAX_ETH_FRAME_SIZE];
	memcpy(ptxb, (char *)packet, (int)length);

	while (len < ETH_ZLEN)
		ptxb[len++] = '\0';

	rtl_flush_buffer(ptxb, ALIGN(len, RTL8169_ALIGN));

	tpc->TxDescArray[entry].buf_Haddr = 0;
#ifdef CONFIG_DM_ETH
	tpc->TxDescArray[entry].buf_addr = cpu_to_le32(
		dm_pci_mem_to_phys(dev, (pci_addr_t)(unsigned long)ptxb));
#else
	tpc->TxDescArray[entry].buf_addr = cpu_to_le32(
		pci_mem_to_phys(dev, (pci_addr_t)(unsigned long)ptxb));
#endif
	if (entry != (NUM_TX_DESC - 1)) {
		tpc->TxDescArray[entry].status =
			cpu_to_le32((OWNbit | FSbit | LSbit) |
				    ((len > ETH_ZLEN) ? len : ETH_ZLEN));
	} else {
		tpc->TxDescArray[entry].status =
			cpu_to_le32((OWNbit | EORbit | FSbit | LSbit) |
				    ((len > ETH_ZLEN) ? len : ETH_ZLEN));
	}
	rtl_flush_tx_desc(&tpc->TxDescArray[entry]);
	if (pplat->device == 0x8125)
		RTL_W8(TxPoll_8125, 0x1);	/* set polling bit */
	else
		RTL_W8(TxPoll_8169, 0x40);	/* set polling bit */

	tpc->cur_tx++;
	to = currticks() + TX_TIMEOUT;
	do {
		rtl_inval_tx_desc(&tpc->TxDescArray[entry]);
	} while ((le32_to_cpu(tpc->TxDescArray[entry].status) & OWNbit)
				&& (currticks() < to));	/* wait */

	if (currticks() >= to) {
#ifdef DEBUG_RTL8169_TX
		puts("tx timeout/error\n");
		printf("%s elapsed time : %lu\n", __func__, currticks()-stime);
#endif
		ret = -ETIMEDOUT;
	} else {
#ifdef DEBUG_RTL8169_TX
		puts("tx done\n");
#endif
		ret = 0;
	}
	/* Delay to make net console (nc) work properly */
	udelay(20);
	return ret;
}

#ifdef CONFIG_DM_ETH
int rtl8169_eth_send(struct udevice *dev, void *packet, int length)
{
	struct rtl8169_private *priv = dev_get_priv(dev);

	return rtl_send_common(dev, priv->iobase, packet, length);
}

#else
static int rtl_send(struct eth_device *dev, void *packet, int length)
{
	return rtl_send_common((pci_dev_t)(unsigned long)dev->priv,
			       dev->iobase, packet, length);
}
#endif

static void rtl8169_set_rx_mode(void)
{
	u32 mc_filter[2];	/* Multicast hash filter */
	int rx_mode;
	u32 tmp = 0;

#ifdef DEBUG_RTL8169
	printf ("%s\n", __FUNCTION__);
#endif

	/* IFF_ALLMULTI */
	/* Too many to filter perfectly -- accept all multicasts. */
	rx_mode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys;
	mc_filter[1] = mc_filter[0] = 0xffffffff;

	tmp = rtl8169_rx_config | rx_mode | (RTL_R32(RxConfig) &
				   rtl_chip_info[tpc->chipset].RxConfigMask);

	RTL_W32(RxConfig, tmp);
	RTL_W32(MAR0 + 0, mc_filter[0]);
	RTL_W32(MAR0 + 4, mc_filter[1]);
}

struct ephy_info {
	unsigned int offset;
	u16 mask;
	u16 bits;
};

struct rtl_cond {
	bool (*check)(void);
	const char *msg;
};

static bool rtl_loop_wait(const struct rtl_cond *c, unsigned long usecs, int n, bool high)
{
	int i;

	for (i = 0; i < n; i++) {
		if (c->check() == high)
			return true;
		udelay(usecs);
	}
	printf("%s == %d (loop: %d, delay: %lu).\n", c->msg, !high, n, usecs);
	return false;
}

/* dali: if the function defined but not used, pls comment out it for compiling. */
#if 0
static bool rtl_loop_wait_high(const struct rtl_cond *c, unsigned long d, int n)
{
	return rtl_loop_wait(c, d, n, true);
}
#endif

static bool rtl_loop_wait_low(const struct rtl_cond *c, unsigned long d, int n)
{
	return rtl_loop_wait(c, d, n, false);
}

/* 
 * Considering of inbox driver's rtl_loop_wait_xxx(&rtl_ephyar_cond, 10, 100) issue, which is fixed by workaround patch
 * (ref. https://lore.kernel.org/lkml/77cf96b2-9a48-ae3f-f234-1c27186b1d3f@amd.com/T/)
		Since increase the loop wait on rtl_ep_ocp_read_cond can eliminate the issue, so let rtl8168ep_driver_start() to wait a bit longer.
		Signed-off-by: Kai-Heng Feng <kai.heng.feng@canonical.com>
 * we can try to use realtek driver v9.013's function to replace it, but still using the inbox driver's function name.
 */
static void rtl_ephy_write(int RegAddr, int value)
{
        int i;

        RTL_W32(EPHYAR,
                EPHYAR_Write |
                (RegAddr & EPHYAR_Reg_Mask_v2) << EPHYAR_Reg_shift |
                (value & EPHYAR_Data_Mask));

        for (i = 0; i < R8125_CHANNEL_WAIT_COUNT; i++) {
                udelay(R8125_CHANNEL_WAIT_TIME);

                /* Check if the RTL8125 has completed EPHY write */
                if (!(RTL_R32(EPHYAR) & EPHYAR_Flag))
                        break;
        }

        udelay(R8125_CHANNEL_EXIT_DELAY_TIME);
}

static u16 rtl_ephy_read(int RegAddr)
{
        int i;
        u16 value = 0xffff;

        RTL_W32(EPHYAR,
                EPHYAR_Read | (RegAddr & EPHYAR_Reg_Mask_v2) << EPHYAR_Reg_shift);

        for (i = 0; i < R8125_CHANNEL_WAIT_COUNT; i++) {
                udelay(R8125_CHANNEL_WAIT_TIME);

                /* Check if the RTL8125 has completed EPHY read */
                if (RTL_R32(EPHYAR) & EPHYAR_Flag) {
                        value = (u16) (RTL_R32(EPHYAR) & EPHYAR_Data_Mask);
                        break;
                }
        }

        udelay(R8125_CHANNEL_EXIT_DELAY_TIME);

        return value;
}

static void __rtl_ephy_init(const struct ephy_info *e, int len)
{
	u16 w;

	while (len-- > 0) {
		w = (rtl_ephy_read(e->offset) & ~e->mask) | e->bits;
		rtl_ephy_write(e->offset, w);
		e++;
	}
}

#define rtl_ephy_init(a) __rtl_ephy_init(a, ARRAY_SIZE(a))

static bool rtl_ocp_reg_failure(u32 reg)
{
	return WARN_ONCE(reg & 0xffff0001, "Invalid ocp reg %x!\n", reg);
}

static void rtl_pcie_state_l2l3_disable(void)
{
	/* work around an issue when PCI reset occurs during L2/L3 state */
	RTL_W8(Config3, RTL_R8(Config3) & ~Rdy_to_L23);
}

static void r8168_mac_ocp_write(u32 reg, u32 data)
{
	if (rtl_ocp_reg_failure(reg))
		return;

	RTL_W32(OCPDR, OCPAR_Flag | (reg << 15) | data);
}

static u16 r8168_mac_ocp_read(u32 reg)
{
	if (rtl_ocp_reg_failure(reg))
		return 0;

	RTL_W32(OCPDR, reg << 15);

	return RTL_R32(OCPDR);
}

static void r8168_mac_ocp_modify(u32 reg, u16 mask, u16 set)
{
	u16 data = r8168_mac_ocp_read(reg);

	r8168_mac_ocp_write(reg, (data & ~mask) | set);
}

static bool rtl_mac_ocp_e00e_cond_check(void)
{
	return r8168_mac_ocp_read(0xe00e) & BIT(13);
}

static const struct rtl_cond rtl_mac_ocp_e00e_cond = {
	.check = rtl_mac_ocp_e00e_cond_check,
	.msg 	 = "rtl_mac_ocp_e00e_cond",
};

bool rtl8125_set_phy_mcu_patch_request(void)
{
        u16 gphy_val;
        u16 WaitCount;
        bool bSuccess = TRUE;

        SetEthPhyOcpBit(0xB820, BIT_4);

        WaitCount = 0;
        do {
                gphy_val = mdio_direct_read_phy_ocp(0xB800);
                udelay(100);
                WaitCount++;
        } while (!(gphy_val & BIT_6) && (WaitCount < 1000));

        if (!(gphy_val & BIT_6) && (WaitCount == 1000)) bSuccess = FALSE;

        if (!bSuccess)
                printf("rtl8125_set_phy_mcu_patch_request fail.\n");

        return bSuccess;
}

bool rtl8125_clear_phy_mcu_patch_request(void)
{
        u16 gphy_val;
        u16 WaitCount;
        bool bSuccess = TRUE;

        ClearEthPhyOcpBit(0xB820, BIT_4);

        WaitCount = 0;
        do {
                gphy_val = mdio_direct_read_phy_ocp(0xB800);
                udelay(100);
                WaitCount++;
        } while ((gphy_val & BIT_6) && (WaitCount < 1000));

        if ((gphy_val & BIT_6) && (WaitCount == 1000)) bSuccess = FALSE;

        if (!bSuccess)
                printf("rtl8125_clear_phy_mcu_patch_request fail.\n");

        return bSuccess;
}

static void rtl_hw_aspm_clkreq_enable(bool enable)
{
	/* Don't enable ASPM in the chip if OS can't control ASPM */
	if (enable) {
		RTL_W8(Config5, RTL_R8(Config5) | ASPM_en);
		RTL_W8(Config2, RTL_R8(Config2) | ClkReqEn);
	} else {
		r8168_mac_ocp_modify(0xe092, 0x00ff, 0);
		RTL_W8(Config2, RTL_R8(Config2) & ~ClkReqEn);
		RTL_W8(Config5, RTL_R8(Config5) & ~ASPM_en);
	}

	udelay(10);
}

/* dali: adding these phy_mcu_patch functions from the realtek driver, to support patching RAM CODE */
#ifdef NEEDS_WrRamCodeToMicroP
static const u16 phy_mcu_ram_code_8125b_2[] = {
        0xa436, 0x8024, 0xa438, 0x3701, 0xa436, 0xB82E, 0xa438, 0x0001,
        0xb820, 0x0090, 0xa436, 0xA016, 0xa438, 0x0000, 0xa436, 0xA012,
        0xa438, 0x0000, 0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010,
        0xa438, 0x1800, 0xa438, 0x801a, 0xa438, 0x1800, 0xa438, 0x803f,
        0xa438, 0x1800, 0xa438, 0x8045, 0xa438, 0x1800, 0xa438, 0x8067,
        0xa438, 0x1800, 0xa438, 0x806d, 0xa438, 0x1800, 0xa438, 0x8071,
        0xa438, 0x1800, 0xa438, 0x80b1, 0xa438, 0xd093, 0xa438, 0xd1c4,
        0xa438, 0x1000, 0xa438, 0x135c, 0xa438, 0xd704, 0xa438, 0x5fbc,
        0xa438, 0xd504, 0xa438, 0xc9f1, 0xa438, 0x1800, 0xa438, 0x0fc9,
        0xa438, 0xbb50, 0xa438, 0xd505, 0xa438, 0xa202, 0xa438, 0xd504,
        0xa438, 0x8c0f, 0xa438, 0xd500, 0xa438, 0x1000, 0xa438, 0x1519,
        0xa438, 0x1000, 0xa438, 0x135c, 0xa438, 0xd75e, 0xa438, 0x5fae,
        0xa438, 0x9b50, 0xa438, 0x1000, 0xa438, 0x135c, 0xa438, 0xd75e,
        0xa438, 0x7fae, 0xa438, 0x1000, 0xa438, 0x135c, 0xa438, 0xd707,
        0xa438, 0x40a7, 0xa438, 0xd719, 0xa438, 0x4071, 0xa438, 0x1800,
        0xa438, 0x1557, 0xa438, 0xd719, 0xa438, 0x2f70, 0xa438, 0x803b,
        0xa438, 0x2f73, 0xa438, 0x156a, 0xa438, 0x5e70, 0xa438, 0x1800,
        0xa438, 0x155d, 0xa438, 0xd505, 0xa438, 0xa202, 0xa438, 0xd500,
        0xa438, 0xffed, 0xa438, 0xd709, 0xa438, 0x4054, 0xa438, 0xa788,
        0xa438, 0xd70b, 0xa438, 0x1800, 0xa438, 0x172a, 0xa438, 0xc0c1,
        0xa438, 0xc0c0, 0xa438, 0xd05a, 0xa438, 0xd1ba, 0xa438, 0xd701,
        0xa438, 0x2529, 0xa438, 0x022a, 0xa438, 0xd0a7, 0xa438, 0xd1b9,
        0xa438, 0xa208, 0xa438, 0x1000, 0xa438, 0x080e, 0xa438, 0xd701,
        0xa438, 0x408b, 0xa438, 0x1000, 0xa438, 0x0a65, 0xa438, 0xf003,
        0xa438, 0x1000, 0xa438, 0x0a6b, 0xa438, 0xd701, 0xa438, 0x1000,
        0xa438, 0x0920, 0xa438, 0x1000, 0xa438, 0x0915, 0xa438, 0x1000,
        0xa438, 0x0909, 0xa438, 0x228f, 0xa438, 0x804e, 0xa438, 0x9801,
        0xa438, 0xd71e, 0xa438, 0x5d61, 0xa438, 0xd701, 0xa438, 0x1800,
        0xa438, 0x022a, 0xa438, 0x2005, 0xa438, 0x091a, 0xa438, 0x3bd9,
        0xa438, 0x0919, 0xa438, 0x1800, 0xa438, 0x0916, 0xa438, 0xd090,
        0xa438, 0xd1c9, 0xa438, 0x1800, 0xa438, 0x1064, 0xa438, 0xd096,
        0xa438, 0xd1a9, 0xa438, 0xd503, 0xa438, 0xa104, 0xa438, 0x0c07,
        0xa438, 0x0902, 0xa438, 0xd500, 0xa438, 0xbc10, 0xa438, 0xd501,
        0xa438, 0xce01, 0xa438, 0xa201, 0xa438, 0x8201, 0xa438, 0xce00,
        0xa438, 0xd500, 0xa438, 0xc484, 0xa438, 0xd503, 0xa438, 0xcc02,
        0xa438, 0xcd0d, 0xa438, 0xaf01, 0xa438, 0xd500, 0xa438, 0xd703,
        0xa438, 0x4371, 0xa438, 0xbd08, 0xa438, 0x1000, 0xa438, 0x135c,
        0xa438, 0xd75e, 0xa438, 0x5fb3, 0xa438, 0xd503, 0xa438, 0xd0f5,
        0xa438, 0xd1c6, 0xa438, 0x0cf0, 0xa438, 0x0e50, 0xa438, 0xd704,
        0xa438, 0x401c, 0xa438, 0xd0f5, 0xa438, 0xd1c6, 0xa438, 0x0cf0,
        0xa438, 0x0ea0, 0xa438, 0x401c, 0xa438, 0xd07b, 0xa438, 0xd1c5,
        0xa438, 0x8ef0, 0xa438, 0x401c, 0xa438, 0x9d08, 0xa438, 0x1000,
        0xa438, 0x135c, 0xa438, 0xd75e, 0xa438, 0x7fb3, 0xa438, 0x1000,
        0xa438, 0x135c, 0xa438, 0xd75e, 0xa438, 0x5fad, 0xa438, 0x1000,
        0xa438, 0x14c5, 0xa438, 0xd703, 0xa438, 0x3181, 0xa438, 0x80af,
        0xa438, 0x60ad, 0xa438, 0x1000, 0xa438, 0x135c, 0xa438, 0xd703,
        0xa438, 0x5fba, 0xa438, 0x1800, 0xa438, 0x0cc7, 0xa438, 0xa802,
        0xa438, 0xa301, 0xa438, 0xa801, 0xa438, 0xc004, 0xa438, 0xd710,
        0xa438, 0x4000, 0xa438, 0x1800, 0xa438, 0x1e79, 0xa436, 0xA026,
        0xa438, 0x1e78, 0xa436, 0xA024, 0xa438, 0x0c93, 0xa436, 0xA022,
        0xa438, 0x1062, 0xa436, 0xA020, 0xa438, 0x0915, 0xa436, 0xA006,
        0xa438, 0x020a, 0xa436, 0xA004, 0xa438, 0x1726, 0xa436, 0xA002,
        0xa438, 0x1542, 0xa436, 0xA000, 0xa438, 0x0fc7, 0xa436, 0xA008,
        0xa438, 0xff00, 0xa436, 0xA016, 0xa438, 0x0010, 0xa436, 0xA012,
        0xa438, 0x0000, 0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010,
        0xa438, 0x1800, 0xa438, 0x801d, 0xa438, 0x1800, 0xa438, 0x802c,
        0xa438, 0x1800, 0xa438, 0x802c, 0xa438, 0x1800, 0xa438, 0x802c,
        0xa438, 0x1800, 0xa438, 0x802c, 0xa438, 0x1800, 0xa438, 0x802c,
        0xa438, 0x1800, 0xa438, 0x802c, 0xa438, 0xd700, 0xa438, 0x6090,
        0xa438, 0x60d1, 0xa438, 0xc95c, 0xa438, 0xf007, 0xa438, 0x60b1,
        0xa438, 0xc95a, 0xa438, 0xf004, 0xa438, 0xc956, 0xa438, 0xf002,
        0xa438, 0xc94e, 0xa438, 0x1800, 0xa438, 0x00cd, 0xa438, 0xd700,
        0xa438, 0x6090, 0xa438, 0x60d1, 0xa438, 0xc95c, 0xa438, 0xf007,
        0xa438, 0x60b1, 0xa438, 0xc95a, 0xa438, 0xf004, 0xa438, 0xc956,
        0xa438, 0xf002, 0xa438, 0xc94e, 0xa438, 0x1000, 0xa438, 0x022a,
        0xa438, 0x1800, 0xa438, 0x0132, 0xa436, 0xA08E, 0xa438, 0xffff,
        0xa436, 0xA08C, 0xa438, 0xffff, 0xa436, 0xA08A, 0xa438, 0xffff,
        0xa436, 0xA088, 0xa438, 0xffff, 0xa436, 0xA086, 0xa438, 0xffff,
        0xa436, 0xA084, 0xa438, 0xffff, 0xa436, 0xA082, 0xa438, 0x012f,
        0xa436, 0xA080, 0xa438, 0x00cc, 0xa436, 0xA090, 0xa438, 0x0103,
        0xa436, 0xA016, 0xa438, 0x0020, 0xa436, 0xA012, 0xa438, 0x0000,
        0xa436, 0xA014, 0xa438, 0x1800, 0xa438, 0x8010, 0xa438, 0x1800,
        0xa438, 0x8020, 0xa438, 0x1800, 0xa438, 0x802a, 0xa438, 0x1800,
        0xa438, 0x8035, 0xa438, 0x1800, 0xa438, 0x803c, 0xa438, 0x1800,
        0xa438, 0x803c, 0xa438, 0x1800, 0xa438, 0x803c, 0xa438, 0x1800,
        0xa438, 0x803c, 0xa438, 0xd107, 0xa438, 0xd042, 0xa438, 0xa404,
        0xa438, 0x1000, 0xa438, 0x09df, 0xa438, 0xd700, 0xa438, 0x5fb4,
        0xa438, 0x8280, 0xa438, 0xd700, 0xa438, 0x6065, 0xa438, 0xd125,
        0xa438, 0xf002, 0xa438, 0xd12b, 0xa438, 0xd040, 0xa438, 0x1800,
        0xa438, 0x077f, 0xa438, 0x0cf0, 0xa438, 0x0c50, 0xa438, 0xd104,
        0xa438, 0xd040, 0xa438, 0x1000, 0xa438, 0x0aa8, 0xa438, 0xd700,
        0xa438, 0x5fb4, 0xa438, 0x1800, 0xa438, 0x0a2e, 0xa438, 0xcb9b,
        0xa438, 0xd110, 0xa438, 0xd040, 0xa438, 0x1000, 0xa438, 0x0b7b,
        0xa438, 0x1000, 0xa438, 0x09df, 0xa438, 0xd700, 0xa438, 0x5fb4,
        0xa438, 0x1800, 0xa438, 0x081b, 0xa438, 0x1000, 0xa438, 0x09df,
        0xa438, 0xd704, 0xa438, 0x7fb8, 0xa438, 0xa718, 0xa438, 0x1800,
        0xa438, 0x074e, 0xa436, 0xA10E, 0xa438, 0xffff, 0xa436, 0xA10C,
        0xa438, 0xffff, 0xa436, 0xA10A, 0xa438, 0xffff, 0xa436, 0xA108,
        0xa438, 0xffff, 0xa436, 0xA106, 0xa438, 0x074d, 0xa436, 0xA104,
        0xa438, 0x0818, 0xa436, 0xA102, 0xa438, 0x0a2c, 0xa436, 0xA100,
        0xa438, 0x077e, 0xa436, 0xA110, 0xa438, 0x000f, 0xa436, 0xb87c,
        0xa438, 0x8625, 0xa436, 0xb87e, 0xa438, 0xaf86, 0xa438, 0x3daf,
        0xa438, 0x8689, 0xa438, 0xaf88, 0xa438, 0x69af, 0xa438, 0x8887,
        0xa438, 0xaf88, 0xa438, 0x9caf, 0xa438, 0x88be, 0xa438, 0xaf88,
        0xa438, 0xbeaf, 0xa438, 0x88be, 0xa438, 0xbf86, 0xa438, 0x49d7,
        0xa438, 0x0040, 0xa438, 0x0277, 0xa438, 0x7daf, 0xa438, 0x2727,
        0xa438, 0x0000, 0xa438, 0x7205, 0xa438, 0x0000, 0xa438, 0x7208,
        0xa438, 0x0000, 0xa438, 0x71f3, 0xa438, 0x0000, 0xa438, 0x71f6,
        0xa438, 0x0000, 0xa438, 0x7229, 0xa438, 0x0000, 0xa438, 0x722c,
        0xa438, 0x0000, 0xa438, 0x7217, 0xa438, 0x0000, 0xa438, 0x721a,
        0xa438, 0x0000, 0xa438, 0x721d, 0xa438, 0x0000, 0xa438, 0x7211,
        0xa438, 0x0000, 0xa438, 0x7220, 0xa438, 0x0000, 0xa438, 0x7214,
        0xa438, 0x0000, 0xa438, 0x722f, 0xa438, 0x0000, 0xa438, 0x7223,
        0xa438, 0x0000, 0xa438, 0x7232, 0xa438, 0x0000, 0xa438, 0x7226,
        0xa438, 0xf8f9, 0xa438, 0xfae0, 0xa438, 0x85b3, 0xa438, 0x3802,
        0xa438, 0xad27, 0xa438, 0x02ae, 0xa438, 0x03af, 0xa438, 0x8830,
        0xa438, 0x1f66, 0xa438, 0xef65, 0xa438, 0xbfc2, 0xa438, 0x1f1a,
        0xa438, 0x96f7, 0xa438, 0x05ee, 0xa438, 0xffd2, 0xa438, 0x00da,
        0xa438, 0xf605, 0xa438, 0xbfc2, 0xa438, 0x2f1a, 0xa438, 0x96f7,
        0xa438, 0x05ee, 0xa438, 0xffd2, 0xa438, 0x00db, 0xa438, 0xf605,
        0xa438, 0xef02, 0xa438, 0x1f11, 0xa438, 0x0d42, 0xa438, 0xbf88,
        0xa438, 0x4202, 0xa438, 0x6e7d, 0xa438, 0xef02, 0xa438, 0x1b03,
        0xa438, 0x1f11, 0xa438, 0x0d42, 0xa438, 0xbf88, 0xa438, 0x4502,
        0xa438, 0x6e7d, 0xa438, 0xef02, 0xa438, 0x1a03, 0xa438, 0x1f11,
        0xa438, 0x0d42, 0xa438, 0xbf88, 0xa438, 0x4802, 0xa438, 0x6e7d,
        0xa438, 0xbfc2, 0xa438, 0x3f1a, 0xa438, 0x96f7, 0xa438, 0x05ee,
        0xa438, 0xffd2, 0xa438, 0x00da, 0xa438, 0xf605, 0xa438, 0xbfc2,
        0xa438, 0x4f1a, 0xa438, 0x96f7, 0xa438, 0x05ee, 0xa438, 0xffd2,
        0xa438, 0x00db, 0xa438, 0xf605, 0xa438, 0xef02, 0xa438, 0x1f11,
        0xa438, 0x0d42, 0xa438, 0xbf88, 0xa438, 0x4b02, 0xa438, 0x6e7d,
        0xa438, 0xef02, 0xa438, 0x1b03, 0xa438, 0x1f11, 0xa438, 0x0d42,
        0xa438, 0xbf88, 0xa438, 0x4e02, 0xa438, 0x6e7d, 0xa438, 0xef02,
        0xa438, 0x1a03, 0xa438, 0x1f11, 0xa438, 0x0d42, 0xa438, 0xbf88,
        0xa438, 0x5102, 0xa438, 0x6e7d, 0xa438, 0xef56, 0xa438, 0xd020,
        0xa438, 0x1f11, 0xa438, 0xbf88, 0xa438, 0x5402, 0xa438, 0x6e7d,
        0xa438, 0xbf88, 0xa438, 0x5702, 0xa438, 0x6e7d, 0xa438, 0xbf88,
        0xa438, 0x5a02, 0xa438, 0x6e7d, 0xa438, 0xe185, 0xa438, 0xa0ef,
        0xa438, 0x0348, 0xa438, 0x0a28, 0xa438, 0x05ef, 0xa438, 0x201b,
        0xa438, 0x01ad, 0xa438, 0x2735, 0xa438, 0x1f44, 0xa438, 0xe085,
        0xa438, 0x88e1, 0xa438, 0x8589, 0xa438, 0xbf88, 0xa438, 0x5d02,
        0xa438, 0x6e7d, 0xa438, 0xe085, 0xa438, 0x8ee1, 0xa438, 0x858f,
        0xa438, 0xbf88, 0xa438, 0x6002, 0xa438, 0x6e7d, 0xa438, 0xe085,
        0xa438, 0x94e1, 0xa438, 0x8595, 0xa438, 0xbf88, 0xa438, 0x6302,
        0xa438, 0x6e7d, 0xa438, 0xe085, 0xa438, 0x9ae1, 0xa438, 0x859b,
        0xa438, 0xbf88, 0xa438, 0x6602, 0xa438, 0x6e7d, 0xa438, 0xaf88,
        0xa438, 0x3cbf, 0xa438, 0x883f, 0xa438, 0x026e, 0xa438, 0x9cad,
        0xa438, 0x2835, 0xa438, 0x1f44, 0xa438, 0xe08f, 0xa438, 0xf8e1,
        0xa438, 0x8ff9, 0xa438, 0xbf88, 0xa438, 0x5d02, 0xa438, 0x6e7d,
        0xa438, 0xe08f, 0xa438, 0xfae1, 0xa438, 0x8ffb, 0xa438, 0xbf88,
        0xa438, 0x6002, 0xa438, 0x6e7d, 0xa438, 0xe08f, 0xa438, 0xfce1,
        0xa438, 0x8ffd, 0xa438, 0xbf88, 0xa438, 0x6302, 0xa438, 0x6e7d,
        0xa438, 0xe08f, 0xa438, 0xfee1, 0xa438, 0x8fff, 0xa438, 0xbf88,
        0xa438, 0x6602, 0xa438, 0x6e7d, 0xa438, 0xaf88, 0xa438, 0x3ce1,
        0xa438, 0x85a1, 0xa438, 0x1b21, 0xa438, 0xad37, 0xa438, 0x341f,
        0xa438, 0x44e0, 0xa438, 0x858a, 0xa438, 0xe185, 0xa438, 0x8bbf,
        0xa438, 0x885d, 0xa438, 0x026e, 0xa438, 0x7de0, 0xa438, 0x8590,
        0xa438, 0xe185, 0xa438, 0x91bf, 0xa438, 0x8860, 0xa438, 0x026e,
        0xa438, 0x7de0, 0xa438, 0x8596, 0xa438, 0xe185, 0xa438, 0x97bf,
        0xa438, 0x8863, 0xa438, 0x026e, 0xa438, 0x7de0, 0xa438, 0x859c,
        0xa438, 0xe185, 0xa438, 0x9dbf, 0xa438, 0x8866, 0xa438, 0x026e,
        0xa438, 0x7dae, 0xa438, 0x401f, 0xa438, 0x44e0, 0xa438, 0x858c,
        0xa438, 0xe185, 0xa438, 0x8dbf, 0xa438, 0x885d, 0xa438, 0x026e,
        0xa438, 0x7de0, 0xa438, 0x8592, 0xa438, 0xe185, 0xa438, 0x93bf,
        0xa438, 0x8860, 0xa438, 0x026e, 0xa438, 0x7de0, 0xa438, 0x8598,
        0xa438, 0xe185, 0xa438, 0x99bf, 0xa438, 0x8863, 0xa438, 0x026e,
        0xa438, 0x7de0, 0xa438, 0x859e, 0xa438, 0xe185, 0xa438, 0x9fbf,
        0xa438, 0x8866, 0xa438, 0x026e, 0xa438, 0x7dae, 0xa438, 0x0ce1,
        0xa438, 0x85b3, 0xa438, 0x3904, 0xa438, 0xac2f, 0xa438, 0x04ee,
        0xa438, 0x85b3, 0xa438, 0x00af, 0xa438, 0x39d9, 0xa438, 0x22ac,
        0xa438, 0xeaf0, 0xa438, 0xacf6, 0xa438, 0xf0ac, 0xa438, 0xfaf0,
        0xa438, 0xacf8, 0xa438, 0xf0ac, 0xa438, 0xfcf0, 0xa438, 0xad00,
        0xa438, 0xf0ac, 0xa438, 0xfef0, 0xa438, 0xacf0, 0xa438, 0xf0ac,
        0xa438, 0xf4f0, 0xa438, 0xacf2, 0xa438, 0xf0ac, 0xa438, 0xb0f0,
        0xa438, 0xacae, 0xa438, 0xf0ac, 0xa438, 0xacf0, 0xa438, 0xacaa,
        0xa438, 0xa100, 0xa438, 0x0ce1, 0xa438, 0x8ff7, 0xa438, 0xbf88,
        0xa438, 0x8402, 0xa438, 0x6e7d, 0xa438, 0xaf26, 0xa438, 0xe9e1,
        0xa438, 0x8ff6, 0xa438, 0xbf88, 0xa438, 0x8402, 0xa438, 0x6e7d,
        0xa438, 0xaf26, 0xa438, 0xf520, 0xa438, 0xac86, 0xa438, 0xbf88,
        0xa438, 0x3f02, 0xa438, 0x6e9c, 0xa438, 0xad28, 0xa438, 0x03af,
        0xa438, 0x3324, 0xa438, 0xad38, 0xa438, 0x03af, 0xa438, 0x32e6,
        0xa438, 0xaf32, 0xa438, 0xfbee, 0xa438, 0x826a, 0xa438, 0x0002,
        0xa438, 0x88a6, 0xa438, 0xaf04, 0xa438, 0x78f8, 0xa438, 0xfaef,
        0xa438, 0x69e0, 0xa438, 0x8015, 0xa438, 0xad20, 0xa438, 0x06bf,
        0xa438, 0x88bb, 0xa438, 0x0275, 0xa438, 0xb1ef, 0xa438, 0x96fe,
        0xa438, 0xfc04, 0xa438, 0x00b8, 0xa438, 0x7a00, 0xa436, 0xb87c,
        0xa438, 0x8ff6, 0xa436, 0xb87e, 0xa438, 0x0705, 0xa436, 0xb87c,
        0xa438, 0x8ff8, 0xa436, 0xb87e, 0xa438, 0x19cc, 0xa436, 0xb87c,
        0xa438, 0x8ffa, 0xa436, 0xb87e, 0xa438, 0x28e3, 0xa436, 0xb87c,
        0xa438, 0x8ffc, 0xa436, 0xb87e, 0xa438, 0x1047, 0xa436, 0xb87c,
        0xa438, 0x8ffe, 0xa436, 0xb87e, 0xa438, 0x0a45, 0xa436, 0xb85e,
        0xa438, 0x271E, 0xa436, 0xb860, 0xa438, 0x3846, 0xa436, 0xb862,
        0xa438, 0x26E6, 0xa436, 0xb864, 0xa438, 0x32E3, 0xa436, 0xb886,
        0xa438, 0x0474, 0xa436, 0xb888, 0xa438, 0xffff, 0xa436, 0xb88a,
        0xa438, 0xffff, 0xa436, 0xb88c, 0xa438, 0xffff, 0xa436, 0xb838,
        0xa438, 0x001f, 0xb820, 0x0010, 0xa436, 0x846e, 0xa438, 0xaf84,
        0xa438, 0x86af, 0xa438, 0x8690, 0xa438, 0xaf86, 0xa438, 0xa4af,
        0xa438, 0x8934, 0xa438, 0xaf89, 0xa438, 0x60af, 0xa438, 0x897e,
        0xa438, 0xaf89, 0xa438, 0xa9af, 0xa438, 0x89a9, 0xa438, 0xee82,
        0xa438, 0x5f00, 0xa438, 0x0284, 0xa438, 0x90af, 0xa438, 0x0441,
        0xa438, 0xf8e0, 0xa438, 0x8ff3, 0xa438, 0xa000, 0xa438, 0x0502,
        0xa438, 0x84a4, 0xa438, 0xae06, 0xa438, 0xa001, 0xa438, 0x0302,
        0xa438, 0x84c8, 0xa438, 0xfc04, 0xa438, 0xf8f9, 0xa438, 0xef59,
        0xa438, 0xe080, 0xa438, 0x15ad, 0xa438, 0x2702, 0xa438, 0xae03,
        0xa438, 0xaf84, 0xa438, 0xc3bf, 0xa438, 0x53ca, 0xa438, 0x0252,
        0xa438, 0xc8ad, 0xa438, 0x2807, 0xa438, 0x0285, 0xa438, 0x2cee,
        0xa438, 0x8ff3, 0xa438, 0x01ef, 0xa438, 0x95fd, 0xa438, 0xfc04,
        0xa438, 0xf8f9, 0xa438, 0xfaef, 0xa438, 0x69bf, 0xa438, 0x53ca,
        0xa438, 0x0252, 0xa438, 0xc8ac, 0xa438, 0x2822, 0xa438, 0xd480,
        0xa438, 0x00bf, 0xa438, 0x8684, 0xa438, 0x0252, 0xa438, 0xa9bf,
        0xa438, 0x8687, 0xa438, 0x0252, 0xa438, 0xa9bf, 0xa438, 0x868a,
        0xa438, 0x0252, 0xa438, 0xa9bf, 0xa438, 0x868d, 0xa438, 0x0252,
        0xa438, 0xa9ee, 0xa438, 0x8ff3, 0xa438, 0x00af, 0xa438, 0x8526,
        0xa438, 0xe08f, 0xa438, 0xf4e1, 0xa438, 0x8ff5, 0xa438, 0xe28f,
        0xa438, 0xf6e3, 0xa438, 0x8ff7, 0xa438, 0x1b45, 0xa438, 0xac27,
        0xa438, 0x0eee, 0xa438, 0x8ff4, 0xa438, 0x00ee, 0xa438, 0x8ff5,
        0xa438, 0x0002, 0xa438, 0x852c, 0xa438, 0xaf85, 0xa438, 0x26e0,
        0xa438, 0x8ff4, 0xa438, 0xe18f, 0xa438, 0xf52c, 0xa438, 0x0001,
        0xa438, 0xe48f, 0xa438, 0xf4e5, 0xa438, 0x8ff5, 0xa438, 0xef96,
        0xa438, 0xfefd, 0xa438, 0xfc04, 0xa438, 0xf8f9, 0xa438, 0xef59,
        0xa438, 0xbf53, 0xa438, 0x2202, 0xa438, 0x52c8, 0xa438, 0xa18b,
        0xa438, 0x02ae, 0xa438, 0x03af, 0xa438, 0x85da, 0xa438, 0xbf57,
        0xa438, 0x7202, 0xa438, 0x52c8, 0xa438, 0xe48f, 0xa438, 0xf8e5,
        0xa438, 0x8ff9, 0xa438, 0xbf57, 0xa438, 0x7502, 0xa438, 0x52c8,
        0xa438, 0xe48f, 0xa438, 0xfae5, 0xa438, 0x8ffb, 0xa438, 0xbf57,
        0xa438, 0x7802, 0xa438, 0x52c8, 0xa438, 0xe48f, 0xa438, 0xfce5,
        0xa438, 0x8ffd, 0xa438, 0xbf57, 0xa438, 0x7b02, 0xa438, 0x52c8,
        0xa438, 0xe48f, 0xa438, 0xfee5, 0xa438, 0x8fff, 0xa438, 0xbf57,
        0xa438, 0x6c02, 0xa438, 0x52c8, 0xa438, 0xa102, 0xa438, 0x13ee,
        0xa438, 0x8ffc, 0xa438, 0x80ee, 0xa438, 0x8ffd, 0xa438, 0x00ee,
        0xa438, 0x8ffe, 0xa438, 0x80ee, 0xa438, 0x8fff, 0xa438, 0x00af,
        0xa438, 0x8599, 0xa438, 0xa101, 0xa438, 0x0cbf, 0xa438, 0x534c,
        0xa438, 0x0252, 0xa438, 0xc8a1, 0xa438, 0x0303, 0xa438, 0xaf85,
        0xa438, 0x77bf, 0xa438, 0x5322, 0xa438, 0x0252, 0xa438, 0xc8a1,
        0xa438, 0x8b02, 0xa438, 0xae03, 0xa438, 0xaf86, 0xa438, 0x64e0,
        0xa438, 0x8ff8, 0xa438, 0xe18f, 0xa438, 0xf9bf, 0xa438, 0x8684,
        0xa438, 0x0252, 0xa438, 0xa9e0, 0xa438, 0x8ffa, 0xa438, 0xe18f,
        0xa438, 0xfbbf, 0xa438, 0x8687, 0xa438, 0x0252, 0xa438, 0xa9e0,
        0xa438, 0x8ffc, 0xa438, 0xe18f, 0xa438, 0xfdbf, 0xa438, 0x868a,
        0xa438, 0x0252, 0xa438, 0xa9e0, 0xa438, 0x8ffe, 0xa438, 0xe18f,
        0xa438, 0xffbf, 0xa438, 0x868d, 0xa438, 0x0252, 0xa438, 0xa9af,
        0xa438, 0x867f, 0xa438, 0xbf53, 0xa438, 0x2202, 0xa438, 0x52c8,
        0xa438, 0xa144, 0xa438, 0x3cbf, 0xa438, 0x547b, 0xa438, 0x0252,
        0xa438, 0xc8e4, 0xa438, 0x8ff8, 0xa438, 0xe58f, 0xa438, 0xf9bf,
        0xa438, 0x547e, 0xa438, 0x0252, 0xa438, 0xc8e4, 0xa438, 0x8ffa,
        0xa438, 0xe58f, 0xa438, 0xfbbf, 0xa438, 0x5481, 0xa438, 0x0252,
        0xa438, 0xc8e4, 0xa438, 0x8ffc, 0xa438, 0xe58f, 0xa438, 0xfdbf,
        0xa438, 0x5484, 0xa438, 0x0252, 0xa438, 0xc8e4, 0xa438, 0x8ffe,
        0xa438, 0xe58f, 0xa438, 0xffbf, 0xa438, 0x5322, 0xa438, 0x0252,
        0xa438, 0xc8a1, 0xa438, 0x4448, 0xa438, 0xaf85, 0xa438, 0xa7bf,
        0xa438, 0x5322, 0xa438, 0x0252, 0xa438, 0xc8a1, 0xa438, 0x313c,
        0xa438, 0xbf54, 0xa438, 0x7b02, 0xa438, 0x52c8, 0xa438, 0xe48f,
        0xa438, 0xf8e5, 0xa438, 0x8ff9, 0xa438, 0xbf54, 0xa438, 0x7e02,
        0xa438, 0x52c8, 0xa438, 0xe48f, 0xa438, 0xfae5, 0xa438, 0x8ffb,
        0xa438, 0xbf54, 0xa438, 0x8102, 0xa438, 0x52c8, 0xa438, 0xe48f,
        0xa438, 0xfce5, 0xa438, 0x8ffd, 0xa438, 0xbf54, 0xa438, 0x8402,
        0xa438, 0x52c8, 0xa438, 0xe48f, 0xa438, 0xfee5, 0xa438, 0x8fff,
        0xa438, 0xbf53, 0xa438, 0x2202, 0xa438, 0x52c8, 0xa438, 0xa131,
        0xa438, 0x03af, 0xa438, 0x85a7, 0xa438, 0xd480, 0xa438, 0x00bf,
        0xa438, 0x8684, 0xa438, 0x0252, 0xa438, 0xa9bf, 0xa438, 0x8687,
        0xa438, 0x0252, 0xa438, 0xa9bf, 0xa438, 0x868a, 0xa438, 0x0252,
        0xa438, 0xa9bf, 0xa438, 0x868d, 0xa438, 0x0252, 0xa438, 0xa9ef,
        0xa438, 0x95fd, 0xa438, 0xfc04, 0xa438, 0xf0d1, 0xa438, 0x2af0,
        0xa438, 0xd12c, 0xa438, 0xf0d1, 0xa438, 0x44f0, 0xa438, 0xd146,
        0xa438, 0xbf86, 0xa438, 0xa102, 0xa438, 0x52c8, 0xa438, 0xbf86,
        0xa438, 0xa102, 0xa438, 0x52c8, 0xa438, 0xd101, 0xa438, 0xaf06,
        0xa438, 0xa570, 0xa438, 0xce42, 0xa438, 0xee83, 0xa438, 0xc800,
        0xa438, 0x0286, 0xa438, 0xba02, 0xa438, 0x8728, 0xa438, 0x0287,
        0xa438, 0xbe02, 0xa438, 0x87f9, 0xa438, 0x0288, 0xa438, 0xc3af,
        0xa438, 0x4771, 0xa438, 0xf8f9, 0xa438, 0xfafb, 0xa438, 0xef69,
        0xa438, 0xfae0, 0xa438, 0x8015, 0xa438, 0xad25, 0xa438, 0x45d2,
        0xa438, 0x0002, 0xa438, 0x8714, 0xa438, 0xac4f, 0xa438, 0x02ae,
        0xa438, 0x0bef, 0xa438, 0x46f6, 0xa438, 0x273c, 0xa438, 0x0400,
        0xa438, 0xab26, 0xa438, 0xae30, 0xa438, 0xe08f, 0xa438, 0xe9e1,
        0xa438, 0x8fea, 0xa438, 0x1b46, 0xa438, 0xab26, 0xa438, 0xef32,
        0xa438, 0x0c31, 0xa438, 0xbf8f, 0xa438, 0xe91a, 0xa438, 0x93d8,
        0xa438, 0x19d9, 0xa438, 0x1b46, 0xa438, 0xab0a, 0xa438, 0x19d8,
        0xa438, 0x19d9, 0xa438, 0x1b46, 0xa438, 0xaa02, 0xa438, 0xae0c,
        0xa438, 0xbf57, 0xa438, 0x1202, 0xa438, 0x58b1, 0xa438, 0xbf57,
        0xa438, 0x1202, 0xa438, 0x58a8, 0xa438, 0xfeef, 0xa438, 0x96ff,
        0xa438, 0xfefd, 0xa438, 0xfc04, 0xa438, 0xf8fb, 0xa438, 0xef79,
        0xa438, 0xa200, 0xa438, 0x08bf, 0xa438, 0x892e, 0xa438, 0x0252,
        0xa438, 0xc8ef, 0xa438, 0x64ef, 0xa438, 0x97ff, 0xa438, 0xfc04,
        0xa438, 0xf8f9, 0xa438, 0xfafb, 0xa438, 0xef69, 0xa438, 0xfae0,
        0xa438, 0x8015, 0xa438, 0xad25, 0xa438, 0x50d2, 0xa438, 0x0002,
        0xa438, 0x878d, 0xa438, 0xac4f, 0xa438, 0x02ae, 0xa438, 0x0bef,
        0xa438, 0x46f6, 0xa438, 0x273c, 0xa438, 0x1000, 0xa438, 0xab31,
        0xa438, 0xae29, 0xa438, 0xe08f, 0xa438, 0xede1, 0xa438, 0x8fee,
        0xa438, 0x1b46, 0xa438, 0xab1f, 0xa438, 0xa200, 0xa438, 0x04ef,
        0xa438, 0x32ae, 0xa438, 0x02d3, 0xa438, 0x010c, 0xa438, 0x31bf,
        0xa438, 0x8fed, 0xa438, 0x1a93, 0xa438, 0xd819, 0xa438, 0xd91b,
        0xa438, 0x46ab, 0xa438, 0x0e19, 0xa438, 0xd819, 0xa438, 0xd91b,
        0xa438, 0x46aa, 0xa438, 0x0612, 0xa438, 0xa205, 0xa438, 0xc0ae,
        0xa438, 0x0cbf, 0xa438, 0x5712, 0xa438, 0x0258, 0xa438, 0xb1bf,
        0xa438, 0x5712, 0xa438, 0x0258, 0xa438, 0xa8fe, 0xa438, 0xef96,
        0xa438, 0xfffe, 0xa438, 0xfdfc, 0xa438, 0x04f8, 0xa438, 0xfbef,
        0xa438, 0x79a2, 0xa438, 0x0005, 0xa438, 0xbf89, 0xa438, 0x1fae,
        0xa438, 0x1ba2, 0xa438, 0x0105, 0xa438, 0xbf89, 0xa438, 0x22ae,
        0xa438, 0x13a2, 0xa438, 0x0205, 0xa438, 0xbf89, 0xa438, 0x25ae,
        0xa438, 0x0ba2, 0xa438, 0x0305, 0xa438, 0xbf89, 0xa438, 0x28ae,
        0xa438, 0x03bf, 0xa438, 0x892b, 0xa438, 0x0252, 0xa438, 0xc8ef,
        0xa438, 0x64ef, 0xa438, 0x97ff, 0xa438, 0xfc04, 0xa438, 0xf8f9,
        0xa438, 0xfaef, 0xa438, 0x69fa, 0xa438, 0xe080, 0xa438, 0x15ad,
        0xa438, 0x2628, 0xa438, 0xe081, 0xa438, 0xabe1, 0xa438, 0x81ac,
        0xa438, 0xef64, 0xa438, 0xbf57, 0xa438, 0x1802, 0xa438, 0x52c8,
        0xa438, 0x1b46, 0xa438, 0xaa0a, 0xa438, 0xbf57, 0xa438, 0x1b02,
        0xa438, 0x52c8, 0xa438, 0x1b46, 0xa438, 0xab0c, 0xa438, 0xbf57,
        0xa438, 0x1502, 0xa438, 0x58b1, 0xa438, 0xbf57, 0xa438, 0x1502,
        0xa438, 0x58a8, 0xa438, 0xfeef, 0xa438, 0x96fe, 0xa438, 0xfdfc,
        0xa438, 0x04f8, 0xa438, 0xf9ef, 0xa438, 0x59f9, 0xa438, 0xe080,
        0xa438, 0x15ad, 0xa438, 0x2622, 0xa438, 0xbf53, 0xa438, 0x2202,
        0xa438, 0x52c8, 0xa438, 0x3972, 0xa438, 0x9e10, 0xa438, 0xe083,
        0xa438, 0xc9ac, 0xa438, 0x2605, 0xa438, 0x0288, 0xa438, 0x2cae,
        0xa438, 0x0d02, 0xa438, 0x8870, 0xa438, 0xae08, 0xa438, 0xe283,
        0xa438, 0xc9f6, 0xa438, 0x36e6, 0xa438, 0x83c9, 0xa438, 0xfdef,
        0xa438, 0x95fd, 0xa438, 0xfc04, 0xa438, 0xf8f9, 0xa438, 0xfafb,
        0xa438, 0xef79, 0xa438, 0xfbbf, 0xa438, 0x5718, 0xa438, 0x0252,
        0xa438, 0xc8ef, 0xa438, 0x64e2, 0xa438, 0x8fe5, 0xa438, 0xe38f,
        0xa438, 0xe61b, 0xa438, 0x659e, 0xa438, 0x10e4, 0xa438, 0x8fe5,
        0xa438, 0xe58f, 0xa438, 0xe6e2, 0xa438, 0x83c9, 0xa438, 0xf636,
        0xa438, 0xe683, 0xa438, 0xc9ae, 0xa438, 0x13e2, 0xa438, 0x83c9,
        0xa438, 0xf736, 0xa438, 0xe683, 0xa438, 0xc902, 0xa438, 0x5820,
        0xa438, 0xef57, 0xa438, 0xe68f, 0xa438, 0xe7e7, 0xa438, 0x8fe8,
        0xa438, 0xffef, 0xa438, 0x97ff, 0xa438, 0xfefd, 0xa438, 0xfc04,
        0xa438, 0xf8f9, 0xa438, 0xfafb, 0xa438, 0xef79, 0xa438, 0xfbe2,
        0xa438, 0x8fe7, 0xa438, 0xe38f, 0xa438, 0xe8ef, 0xa438, 0x65e2,
        0xa438, 0x81b8, 0xa438, 0xe381, 0xa438, 0xb9ef, 0xa438, 0x7502,
        0xa438, 0x583b, 0xa438, 0xac50, 0xa438, 0x1abf, 0xa438, 0x5718,
        0xa438, 0x0252, 0xa438, 0xc8ef, 0xa438, 0x64e2, 0xa438, 0x8fe5,
        0xa438, 0xe38f, 0xa438, 0xe61b, 0xa438, 0x659e, 0xa438, 0x1ce4,
        0xa438, 0x8fe5, 0xa438, 0xe58f, 0xa438, 0xe6ae, 0xa438, 0x0cbf,
        0xa438, 0x5715, 0xa438, 0x0258, 0xa438, 0xb1bf, 0xa438, 0x5715,
        0xa438, 0x0258, 0xa438, 0xa8e2, 0xa438, 0x83c9, 0xa438, 0xf636,
        0xa438, 0xe683, 0xa438, 0xc9ff, 0xa438, 0xef97, 0xa438, 0xfffe,
        0xa438, 0xfdfc, 0xa438, 0x04f8, 0xa438, 0xf9fa, 0xa438, 0xef69,
        0xa438, 0xe080, 0xa438, 0x15ad, 0xa438, 0x264b, 0xa438, 0xbf53,
        0xa438, 0xca02, 0xa438, 0x52c8, 0xa438, 0xad28, 0xa438, 0x42bf,
        0xa438, 0x8931, 0xa438, 0x0252, 0xa438, 0xc8ef, 0xa438, 0x54bf,
        0xa438, 0x576c, 0xa438, 0x0252, 0xa438, 0xc8a1, 0xa438, 0x001b,
        0xa438, 0xbf53, 0xa438, 0x4c02, 0xa438, 0x52c8, 0xa438, 0xac29,
        0xa438, 0x0dac, 0xa438, 0x2805, 0xa438, 0xa302, 0xa438, 0x16ae,
        0xa438, 0x20a3, 0xa438, 0x0311, 0xa438, 0xae1b, 0xa438, 0xa304,
        0xa438, 0x0cae, 0xa438, 0x16a3, 0xa438, 0x0802, 0xa438, 0xae11,
        0xa438, 0xa309, 0xa438, 0x02ae, 0xa438, 0x0cbf, 0xa438, 0x5715,
        0xa438, 0x0258, 0xa438, 0xb1bf, 0xa438, 0x5715, 0xa438, 0x0258,
        0xa438, 0xa8ef, 0xa438, 0x96fe, 0xa438, 0xfdfc, 0xa438, 0x04f0,
        0xa438, 0xa300, 0xa438, 0xf0a3, 0xa438, 0x02f0, 0xa438, 0xa304,
        0xa438, 0xf0a3, 0xa438, 0x06f0, 0xa438, 0xa308, 0xa438, 0xf0a2,
        0xa438, 0x8074, 0xa438, 0xa600, 0xa438, 0xac4f, 0xa438, 0x02ae,
        0xa438, 0x0bef, 0xa438, 0x46f6, 0xa438, 0x273c, 0xa438, 0x1000,
        0xa438, 0xab1b, 0xa438, 0xae16, 0xa438, 0xe081, 0xa438, 0xabe1,
        0xa438, 0x81ac, 0xa438, 0x1b46, 0xa438, 0xab0c, 0xa438, 0xac32,
        0xa438, 0x04ef, 0xa438, 0x32ae, 0xa438, 0x02d3, 0xa438, 0x04af,
        0xa438, 0x486c, 0xa438, 0xaf48, 0xa438, 0x82af, 0xa438, 0x4888,
        0xa438, 0xe081, 0xa438, 0x9be1, 0xa438, 0x819c, 0xa438, 0xe28f,
        0xa438, 0xe3ad, 0xa438, 0x3009, 0xa438, 0x1f55, 0xa438, 0xe38f,
        0xa438, 0xe20c, 0xa438, 0x581a, 0xa438, 0x45e4, 0xa438, 0x83a6,
        0xa438, 0xe583, 0xa438, 0xa7af, 0xa438, 0x2a75, 0xa438, 0xe08f,
        0xa438, 0xe3ad, 0xa438, 0x201c, 0xa438, 0x1f44, 0xa438, 0xe18f,
        0xa438, 0xe10c, 0xa438, 0x44ef, 0xa438, 0x64e0, 0xa438, 0x8232,
        0xa438, 0xe182, 0xa438, 0x331b, 0xa438, 0x649f, 0xa438, 0x091f,
        0xa438, 0x44e1, 0xa438, 0x8fe2, 0xa438, 0x0c48, 0xa438, 0x1b54,
        0xa438, 0xe683, 0xa438, 0xa6e7, 0xa438, 0x83a7, 0xa438, 0xaf2b,
        0xa438, 0xd900, 0xa436, 0xb818, 0xa438, 0x043d, 0xa436, 0xb81a,
        0xa438, 0x06a3, 0xa436, 0xb81c, 0xa438, 0x476d, 0xa436, 0xb81e,
        0xa438, 0x4852, 0xa436, 0xb850, 0xa438, 0x2A69, 0xa436, 0xb852,
        0xa438, 0x2BD3, 0xa436, 0xb878, 0xa438, 0xffff, 0xa436, 0xb884,
        0xa438, 0xffff, 0xa436, 0xb832, 0xa438, 0x003f, 0xb844, 0xffff,
        0xa436, 0x8fe9, 0xa438, 0x0000, 0xa436, 0x8feb, 0xa438, 0x02fe,
        0xa436, 0x8fed, 0xa438, 0x0019, 0xa436, 0x8fef, 0xa438, 0x0bdb,
        0xa436, 0x8ff1, 0xa438, 0x0ca4, 0xa436, 0x0000, 0xa438, 0x0000,
        0xa436, 0xB82E, 0xa438, 0x0000, 0xa436, 0x8024, 0xa438, 0x0000,
        0xa436, 0x801E, 0xa438, 0x0024, 0xb820, 0x0000, 0xFFFF, 0xFFFF
};

static void
rtl8125_set_phy_mcu_ram_code(const u16 *ramcode, u16 codesize)
{
        u16 i;
        u16 addr;
        u16 val;

        if (ramcode == NULL || codesize % 2) {
                goto out;
        }

        for (i = 0; i < codesize; i += 2) {
                addr = ramcode[i];
                val = ramcode[i + 1];
                if (addr == 0xFFFF && val == 0xFFFF) {
                        break;
                }
                mdio_direct_write_phy_ocp(addr, val);
        }

out:
        return;
}

static void
rtl8125_real_set_phy_mcu_8125b_2(void)
{
        rtl8125_set_phy_mcu_ram_code(phy_mcu_ram_code_8125b_2,
                                     ARRAY_SIZE(phy_mcu_ram_code_8125b_2));
}

static void
rtl8125_set_phy_mcu_8125b_2(void)
{
        rtl8125_set_phy_mcu_patch_request();
        rtl8125_real_set_phy_mcu_8125b_2();
        rtl8125_clear_phy_mcu_patch_request();
}

static void
rtl8125_write_hw_phy_mcu_code_ver(void)
{
	mdio_direct_write_phy_ocp(0xA436, 0x801E);
	mdio_direct_write_phy_ocp(0xA438, tpc->sw_ram_code_ver);
	tpc->hw_ram_code_ver = tpc->sw_ram_code_ver;
}

static void
rtl8125_init_hw_phy_mcu(void)
{
	rtl8125_set_phy_mcu_8125b_2();
	rtl8125_write_hw_phy_mcu_code_ver();
	rtl8125_mdio_write(0x1F, 0x0000);
	tpc->HwHasWrRamCodeToMicroP = 0x1;
}

static void
rtl8125_set_hw_phy_before_init_phy_mcu(void){
	/* dali: according to realtek driver, 8125b is CFG_METHOD_5, and it doesn't need to do this function. */
	return;
}
#endif    //dali: end of NEEDS_WrRamCodeToMicroP macro

static bool rtl8125_is_adv_eee_enabled(void)
{
	if (mdio_direct_read_phy_ocp(0xA430) & BIT_15)
		return true;
	return false;
}

static void rtl8125_disable_adv_eee(void){
		bool lock;

        if (rtl8125_is_adv_eee_enabled())
                lock = true;
        else
                lock = false;

        if (lock)
                rtl8125_set_phy_mcu_patch_request();

        ClearMcuAccessRegBit(0xE052, BIT_0);
        ClearEthPhyOcpBit(0xA442, BIT_12 | BIT_13);
        ClearEthPhyOcpBit(0xA430, BIT_15);

        if (lock)
                rtl8125_clear_phy_mcu_patch_request();
}

static void rtl_hw_start_8125_common(void)
{
	rtl_pcie_state_l2l3_disable();

	RTL_W16(0x382, 0x221b);
	RTL_W8(0x4500, 0);
	RTL_W16(0x4800, 0);

	/* disable UPS */
	//dali: Considering ups has been disabled here, so the driver doesn't need to support ups if we do not need this feature.
	//dali: Delete rtl8125_wait_phy_ups_resume().
	r8168_mac_ocp_modify(0xd40a, 0x0010, 0x0000);

	RTL_W8(Config1, RTL_R8(Config1) & ~0x10);

	r8168_mac_ocp_write(0xc140, 0xffff);
	r8168_mac_ocp_write(0xc142, 0xffff);

	r8168_mac_ocp_modify(0xd3e2, 0x0fff, 0x03a9);
	r8168_mac_ocp_modify(0xd3e4, 0x00ff, 0x0000);
	r8168_mac_ocp_modify(0xe860, 0x0000, 0x0080);

	/* disable new tx descriptor format */
	r8168_mac_ocp_modify(0xeb58, 0x0001, 0x0000);

	r8168_mac_ocp_modify(0xe614, 0x0700, 0x0200);
	r8168_mac_ocp_modify(0xe63e, 0x0c30, 0x0000);
	r8168_mac_ocp_modify(0xc0b4, 0x0000, 0x000c);
	r8168_mac_ocp_modify(0xeb6a, 0x00ff, 0x0033);
	r8168_mac_ocp_modify(0xeb50, 0x03e0, 0x0040);
	r8168_mac_ocp_modify(0xe056, 0x00f0, 0x0030);
	r8168_mac_ocp_modify(0xe040, 0x1000, 0x0000);
	r8168_mac_ocp_modify(0xea1c, 0x0003, 0x0001);
	r8168_mac_ocp_modify(0xea1c, 0x0004, 0x0000);
	r8168_mac_ocp_modify(0xe0c0, 0x4f0f, 0x4403);
	r8168_mac_ocp_modify(0xe052, 0x0080, 0x0068);
	r8168_mac_ocp_modify(0xd430, 0x0fff, 0x047f);

	r8168_mac_ocp_modify(0xea1c, 0x0004, 0x0000);
	r8168_mac_ocp_modify(0xeb54, 0x0000, 0x0001);
	udelay(1);
	r8168_mac_ocp_modify(0xeb54, 0x0001, 0x0000);
	RTL_W16(0x1880, RTL_R16(0x1880) & ~0x0030);

	r8168_mac_ocp_write(0xe098, 0xc302);

	/* Keep this function until we can clearly understand the meaning of this mac ocp 0xE00E register */
	rtl_loop_wait_low(&rtl_mac_ocp_e00e_cond, 1000, 10);

	/* dali: from the inbox driver. 
	 * But considering we have disabled eee in the uboot driver, so we didn't need to modify eee config.
	 * So we delete rtl8125b_config_eee_mac().
	 */
	
	/* dali: from the inbox driver.
	 * Rtl_disable_rxdvgate() is be executed in the rtl_hw_start_8125_common whose call flow is:
		rtl8169_eth_start -> rtl8169_common_start
		rtl8169_eth_probe -> rtl_init
			rtl_hw_start_8125
				rtl_hw_start_8125b
					rtl_hw_start_8125_common
		And at the end of rtl8169_eth_probe() func, disable rxdvgate has been done.
		So there is no need to call rtl_disable_rxdvgate() in the rtl_init() func.
		So we delete rtl_disable_rxdvgate();		
	 */
}

static void rtl_hw_start_8125b(void)
{
	static const struct ephy_info e_info_8125b[] = {
		{ 0x0b, 0xffff, 0xa908 },
		{ 0x1e, 0xffff, 0x20eb },
		{ 0x4b, 0xffff, 0xa908 },
		{ 0x5e, 0xffff, 0x20eb },
		{ 0x22, 0x0030, 0x0020 },
		{ 0x62, 0x0030, 0x0020 },
	};
	/* Considering that the uboot driver may not need L0s/L1, the following code is removed */
	//rtl_set_def_aspm_entry_latency();
	
	/* adding: disable aspm and clock request before ephy access */
	rtl_hw_aspm_clkreq_enable(false);
	rtl_ephy_init(e_info_8125b);
	rtl_hw_start_8125_common();
	rtl_hw_aspm_clkreq_enable(true);
}

static void rtl_hw_start_8125(void)
{
	int i;

	RTL_W8(INT_CFG0_8125, 0x00);

	/* disable interrupt coalescing */
	for (i = 0xa00; i < 0xa80; i += 4)
		RTL_W32(i, 0);
	RTL_W16(INT_CFG1_8125, 0x0000);

	rtl_hw_start_8125b();
}

static void rtl8125b_config_eee_phy(void)
{	
	//dali: add these from rtk driver rtl8125_disable_eee
	ClearMcuAccessRegBit(0xE040, (BIT_1 | BIT_0));
	SetEthPhyOcpBit(0xA432, BIT_4);
	ClearEthPhyOcpBit(0xA5D0, (BIT_2 | BIT_1));
	//end

	ClearAndSetEthPhyOcpBit(0xa6d4, 0x0001, 0x0000);
	ClearAndSetEthPhyOcpBit(0xa6d8, 0x0010, 0x0000);
	ClearAndSetEthPhyOcpBit(0xa428, 0x0080, 0x0000);
	ClearAndSetEthPhyOcpBit(0xa4a2, 0x0200, 0x0000);
	
	//dali: add these from rtk driver rtl8125_disable_eee
	/*Advanced EEE*/
	rtl8125_disable_adv_eee();
}

//dali: follow the inbox driver(r8169_phy_config.c), and change the name "rtl8125_hw_phy_config_8125b_2" to "rtl8125b_hw_phy_config"
static void rtl8125b_hw_phy_config(void)
{
        SetEthPhyOcpBit(0xA442, BIT_11);
        ClearAndSetEthPhyOcpBit(0xAC46, 0x00F0, 0x0090);
        ClearAndSetEthPhyOcpBit(0xAD30, 0x0003, 0x0001);

        mdio_direct_write_phy_ocp(0xB87C, 0x80F5);
        mdio_direct_write_phy_ocp(0xB87E, 0x760E);
        mdio_direct_write_phy_ocp(0xB87C, 0x8107);
        mdio_direct_write_phy_ocp(0xB87E, 0x360E);
        mdio_direct_write_phy_ocp(0xB87C, 0x8551);
        ClearAndSetEthPhyOcpBit(0xB87E,
                                BIT_15 | BIT_14 | BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_9 | BIT_8,
                                BIT_11);

        ClearAndSetEthPhyOcpBit(0xbf00, 0xE000, 0xA000);
        ClearAndSetEthPhyOcpBit(0xbf46, 0x0F00, 0x0300);

        mdio_direct_write_phy_ocp(0xa436, 0x8044);
        mdio_direct_write_phy_ocp(0xa438, 0x2417);
        mdio_direct_write_phy_ocp(0xa436, 0x804A);
        mdio_direct_write_phy_ocp(0xa438, 0x2417);
        mdio_direct_write_phy_ocp(0xa436, 0x8050);
        mdio_direct_write_phy_ocp(0xa438, 0x2417);
        mdio_direct_write_phy_ocp(0xa436, 0x8056);
        mdio_direct_write_phy_ocp(0xa438, 0x2417);
        mdio_direct_write_phy_ocp(0xa436, 0x805C);
        mdio_direct_write_phy_ocp(0xa438, 0x2417);
        mdio_direct_write_phy_ocp(0xa436, 0x8062);
        mdio_direct_write_phy_ocp(0xa438, 0x2417);
        mdio_direct_write_phy_ocp(0xa436, 0x8068);
        mdio_direct_write_phy_ocp(0xa438, 0x2417);
        mdio_direct_write_phy_ocp(0xa436, 0x806E);
        mdio_direct_write_phy_ocp(0xa438, 0x2417);
        mdio_direct_write_phy_ocp(0xa436, 0x8074);
        mdio_direct_write_phy_ocp(0xa438, 0x2417);
        mdio_direct_write_phy_ocp(0xa436, 0x807A);
        mdio_direct_write_phy_ocp(0xa438, 0x2417);
        SetEthPhyOcpBit(0xA4CA, BIT_6);
        ClearAndSetEthPhyOcpBit(0xBF84,
                                BIT_15 | BIT_14 | BIT_13,
                                BIT_15 | BIT_13);

		//dali: inbox's rtl8125_legacy_force_mode
		ClearEthPhyOcpBit(0xa5b4, BIT_15);
		
		rtl8125b_config_eee_phy();
}


static int rtl_nway_reset(void)
{
        int ret, bmcr;

        /* if autoneg is off, it's an error */
        rtl8125_mdio_write(0x1F, 0x0000);
        bmcr = rtl8125_mdio_read(MII_BMCR);

        if (bmcr & BMCR_ANENABLE) {
                bmcr |= BMCR_ANRESTART;
                rtl8125_mdio_write(MII_BMCR, bmcr);
                ret = 0;
        } else {
                ret = -EINVAL;
        }

        return ret;
}



//dali: follow the inbox driver, and change the name "rtl8125_hw_phy_config" to "rtl8169_init_phy"
/*
 * this function is used to setting: gphy(/gphy ocp) param, disable eee_phy
 */
static void rtl8169_init_phy(void)
{
	int val = 0;
	
	/* dali: inbox driver's rtl8125b_hw_phy_config() ~= rtk driver's rtl8125_hw_phy_config_8125b_2().
	 * If we follow the realtek driver code style, we need to add these before hw_phy_config():
			rtl8125_set_hw_phy_before_init_phy_mcu()
			rtl8125_init_hw_phy_mcu()
		These function is used to patch phy mcu code(a.k.a. PHY MCU RAM CODE), such as SUZHOU/PEGATRON happy new year issue.
		
		However, we used an external card to test on the experimental platform and found that the network card does not need to be patched.
		So we use a macro "NEEDS_WrRamCodeToMicroP" switch to enable/disable it. And set the default setting to "undef".
	 */
	#ifdef NEEDS_WrRamCodeToMicroP
		rtl8125_set_hw_phy_before_init_phy_mcu();
		rtl8125_init_hw_phy_mcu();
	#endif
	 
	rtl8125b_hw_phy_config();
	
	/*dali: phy_reset_enable() called in rtk driver, which will do BMCR_RESET|BMCR_ANENABLE.
	 *we also using rtl_nway_reset() to replace it, which will do BMCR_ANRESTART only. They have the same effect on restarting.
	 */
	rtl_nway_reset();
	
	/*
	 * dali: In fact, it doesn’t matter whether the auto-negotiation is completed here.
	 * Because when leaving rtl8169_init_phy(), the rtl_init() will do these work too.
	 */
	/* Support auto-negotiation Gigabit network */
	//dali: ANAR
	val = rtl8125_mdio_read(0x4);
	rtl8125_mdio_write(0x4, val | BIT_5 | BIT_6 | BIT_7 | BIT_8 | BIT_10 | BIT_11);
	
	//dali: GBCR
	val = rtl8125_mdio_read(0x9);
	rtl8125_mdio_write(0x9, val | BIT_9);

	//dali: BMCR, Restart_AN
	val = rtl8125_mdio_read(0x0);
	rtl8125_mdio_write(0x0, val | BIT_9);

	rtl8125_mdio_write(0x1F, 0x0000);

}

static void rtl8125_phy_power_up(void)
{
	rtl8125_mdio_write(0x1F, 0x0000);
	rtl8125_mdio_write(MII_BMCR, BMCR_ANENABLE);
}

static void rtl8125_powerup_pll(void)
{
	rtl8125_phy_power_up();
}

#ifdef CONFIG_DM_ETH
static void rtl8169_hw_start(struct udevice *dev)
#else
static void rtl8169_hw_start(pci_dev_t dev)
#endif
{
	u32 i;

#ifdef DEBUG_RTL8169
	int stime = currticks();
	printf ("%s\n", __FUNCTION__);
#endif

#if 0
	/* Soft reset the chip. */
	RTL_W8(ChipCmd, CmdReset);

	/* Check that the chip has finished the reset. */
	for (i = 1000; i > 0; i--) {
		if ((RTL_R8(ChipCmd) & CmdReset) == 0)
			break;
		else
			udelay(10);
	}
#endif

	RTL_W8(Cfg9346, Cfg9346_Unlock);

	/* RTL-8169sb/8110sb or previous version */
	if (tpc->chipset <= 5)
		RTL_W8(ChipCmd, CmdTxEnb | CmdRxEnb);

	RTL_W8(EarlyTxThres, EarlyTxThld);

	/* For gigabit rtl8169 */
	RTL_W16(RxMaxSize, RxPacketMaxSize);

	/* Set Rx Config register */
	i = rtl8169_rx_config | (RTL_R32(RxConfig) &
				 rtl_chip_info[tpc->chipset].RxConfigMask);
	RTL_W32(RxConfig, i);

	/* Set DMA burst size and Interframe Gap Time */
	RTL_W32(TxConfig, (TX_DMA_BURST << TxDMAShift) |
				(InterFrameGap << TxInterFrameGapShift));


	tpc->cur_rx = 0;

#ifdef CONFIG_DM_ETH
	RTL_W32(TxDescStartAddrLow, dm_pci_mem_to_phys(dev,
			(pci_addr_t)(unsigned long)tpc->TxDescArray));
#else
	RTL_W32(TxDescStartAddrLow, pci_mem_to_phys(dev,
			(pci_addr_t)(unsigned long)tpc->TxDescArray));
#endif
	RTL_W32(TxDescStartAddrHigh, (unsigned long)0);
#ifdef CONFIG_DM_ETH
	RTL_W32(RxDescStartAddrLow, dm_pci_mem_to_phys(
			dev, (pci_addr_t)(unsigned long)tpc->RxDescArray));
#else
	RTL_W32(RxDescStartAddrLow, pci_mem_to_phys(
			dev, (pci_addr_t)(unsigned long)tpc->RxDescArray));
#endif
	RTL_W32(RxDescStartAddrHigh, (unsigned long)0);

	/* RTL-8169sc/8110sc or later version */
	if (tpc->chipset > 5)
		RTL_W8(ChipCmd, CmdTxEnb | CmdRxEnb);

	RTL_W8(Cfg9346, Cfg9346_Lock);
	udelay(10);

	RTL_W32(RxMissed, 0);

	rtl8169_set_rx_mode();

	/* no early-rx interrupts */
	RTL_W16(MultiIntr, RTL_R16(MultiIntr) & 0xF000);

#ifdef DEBUG_RTL8169
	printf("%s elapsed time : %lu\n", __func__, currticks()-stime);
#endif
}

#ifdef CONFIG_DM_ETH
static void rtl8169_init_ring(struct udevice *dev)
#else
static void rtl8169_init_ring(pci_dev_t dev)
#endif
{
	int i;

#ifdef DEBUG_RTL8169
	int stime = currticks();
	printf ("%s\n", __FUNCTION__);
#endif

	tpc->cur_rx = 0;
	tpc->cur_tx = 0;
	tpc->dirty_tx = 0;
	memset(tpc->TxDescArray, 0x0, NUM_TX_DESC * sizeof(struct TxDesc));
	memset(tpc->RxDescArray, 0x0, NUM_RX_DESC * sizeof(struct RxDesc));

	for (i = 0; i < NUM_TX_DESC; i++) {
		tpc->Tx_skbuff[i] = &txb[i];
	}

	for (i = 0; i < NUM_RX_DESC; i++) {
		if (i == (NUM_RX_DESC - 1))
			tpc->RxDescArray[i].status =
				cpu_to_le32((OWNbit | EORbit) + RX_BUF_SIZE);
		else
			tpc->RxDescArray[i].status =
				cpu_to_le32(OWNbit + RX_BUF_SIZE);

		tpc->RxBufferRing[i] = &rxb[i * RX_BUF_SIZE];
#ifdef CONFIG_DM_ETH
		tpc->RxDescArray[i].buf_addr = cpu_to_le32(dm_pci_mem_to_phys(
			dev, (pci_addr_t)(unsigned long)tpc->RxBufferRing[i]));
#else
		tpc->RxDescArray[i].buf_addr = cpu_to_le32(pci_mem_to_phys(
			dev, (pci_addr_t)(unsigned long)tpc->RxBufferRing[i]));
#endif
		rtl_flush_rx_desc(&tpc->RxDescArray[i]);
	}

#ifdef DEBUG_RTL8169
	printf("%s elapsed time : %lu\n", __func__, currticks()-stime);
#endif
}

#ifdef CONFIG_DM_ETH
static void rtl8169_common_start(struct udevice *dev, unsigned char *enetaddr,
				 unsigned long dev_iobase)
#else
static void rtl8169_common_start(pci_dev_t dev, unsigned char *enetaddr,
				 unsigned long dev_iobase)
#endif
{
	int i;

#ifdef DEBUG_RTL8169
	int stime = currticks();
	printf ("%s\n", __FUNCTION__);
#endif

	ioaddr = dev_iobase;

	rtl8169_init_ring(dev);

	/* For the initialization of the network card chip(eg. ephy param.) */
	if(tpc->chipset == 16){
		/* dali: Considering that rtl_hw_start_8125() has already been executed in the rtl8169_eth_probe() period,
		 * so i don't think there's any need to repeat it, especially considering the execution order of the driver code about .probe/.start
		 * so we delete: rtl_hw_start_8125();
		 */
		 //rtl_hw_start_8125();
	} else{
		/* you can add 8111 series EXTRA ephy function here. */
	}
	
	rtl8169_hw_start(dev);
	/* Construct a perfect filter frame with the mac address as first match
	 * and broadcast for all others */
	for (i = 0; i < 192; i++)
		txb[i] = 0xFF;

	txb[0] = enetaddr[0];
	txb[1] = enetaddr[1];
	txb[2] = enetaddr[2];
	txb[3] = enetaddr[3];
	txb[4] = enetaddr[4];
	txb[5] = enetaddr[5];

#ifdef DEBUG_RTL8169
	printf("%s elapsed time : %lu\n", __func__, currticks()-stime);
#endif
}

#ifdef CONFIG_DM_ETH
static int rtl8169_eth_start(struct udevice *dev)
{
	#ifdef RK_v201709
		struct eth_pdata *plat = dev_get_platdata(dev);
	#else
		struct eth_pdata *plat = dev_get_plat(dev);
	#endif

	struct rtl8169_private *priv = dev_get_priv(dev);

	rtl8169_common_start(dev, plat->enetaddr, priv->iobase);

	return 0;
}
#else
/**************************************************************************
RESET - Finish setting up the ethernet interface
***************************************************************************/
static int rtl_reset(struct eth_device *dev, struct bd_info *bis)
{
	rtl8169_common_start((pci_dev_t)(unsigned long)dev->priv,
			     dev->enetaddr, dev->iobase);

	return 0;
}
#endif /* nCONFIG_DM_ETH */

#ifdef CONFIG_DM_ETH
static void rtl_halt_common(struct udevice *dev)
{
	struct rtl8169_private *priv = dev_get_priv(dev);
	ioaddr = priv->iobase;
#else
static void rtl_halt_common(struct eth_device *dev)
{
	ioaddr = dev->iobase;
#endif

#ifdef DEBUG_RTL8169
	printf ("%s\n", __FUNCTION__);
#endif

	int i;

	/* Stop the chip's Tx and Rx DMA processes. */
	RTL_W8(ChipCmd, 0x00);

	/* Disable interrupts by clearing the interrupt mask. */
	if(tpc->chipset == 16)
		RTL_W16(IntrMask_8125, 0x0000);
	else
		RTL_W16(IntrMask_8169, 0x0000);

	RTL_W32(RxMissed, 0);

	for (i = 0; i < NUM_RX_DESC; i++) {
		/* dali:
		 * If you encounter the problem of transmission pause(a.k.a. TxHang) when using tftpd32 software, you can
		 * delete this line of code(tpc->RxBufferRing[i] = NULL;) and you may get a surprise.
		 */
		tpc->RxBufferRing[i] = NULL;
	}
}

#ifdef CONFIG_DM_ETH
void rtl8169_eth_stop(struct udevice *dev)
{
	rtl_halt_common(dev);
}
#else
/**************************************************************************
HALT - Turn off ethernet interface
***************************************************************************/
static void rtl_halt(struct eth_device *dev)
{
	rtl_halt_common(dev);
}
#endif

#ifdef CONFIG_DM_ETH
static int rtl8169_write_hwaddr(struct udevice *dev)
{
	#ifdef RK_v201709
		struct eth_pdata *plat = dev_get_platdata(dev);
	#else
		struct eth_pdata *plat = dev_get_plat(dev);
	#endif

	unsigned int i;

	#ifdef DEBUG_RTL8169
		printf("\n%s enetaddr = %pM\n", __FUNCTION__, plat->enetaddr);
	#endif

	/*dali: pls. you need to unlock first, and modify the macio reg, and then lock again*/
	/*set mac address*/
	RTL_W8(Cfg9346, Cfg9346_Unlock);

	for (i = 0; i < MAC_ADDR_LEN; i++)
		RTL_W8(MAC0 + i, plat->enetaddr[i]);

	RTL_W8(Cfg9346, Cfg9346_Lock);

	#ifdef DEBUG_RTL8169
		unsigned char read_enetaddr[ARP_HLEN];
		/* Get MAC address. FIXME: read EEPROM */
		for (int i = 0; i < MAC_ADDR_LEN; i++)
				read_enetaddr[i] = RTL_R8(MAC0 + i);
		printf ("get mac address: %pM\n", read_enetaddr);
	#endif

	return 0;
}

#endif

/**************************************************************************
INIT - Look for an adapter, this routine's visible to the outside
***************************************************************************/

#define board_found 1
#define valid_link 0
static int rtl_init(unsigned long dev_ioaddr, const char *name,
		    unsigned char *enetaddr)
{
	static int board_idx = -1;
	int i, rc;
	int option = -1, Cap10_100 = 0, Cap1000 = 0;

#ifdef DEBUG_RTL8169
	printf ("%s\n", __FUNCTION__);
#endif
	ioaddr = dev_ioaddr;

	board_idx++;

	/* point to private storage */
	tpc = &tpx;

	rc = rtl8169_init_board(ioaddr, name);
	if (rc)
		return rc;

	/* Get MAC address.  FIXME: read EEPROM */
	for (i = 0; i < MAC_ADDR_LEN; i++)
		enetaddr[i] = RTL_R8(MAC0 + i);

#ifdef DEBUG_RTL8169
	printf("chipset = %d\n", tpc->chipset);
	printf("MAC Address");
	for (i = 0; i < MAC_ADDR_LEN; i++)
		printf(":%02x", enetaddr[i]);
	putc('\n');
#endif

#ifdef DEBUG_RTL8169
	/* Print out some hardware info */
	printf("%s: at ioaddr 0x%lx\n", name, ioaddr);
#endif

	if(tpc->chipset == 16){
		/* you can add 8125b customize ledcfg function here.*/
		rtl_hw_start_8125();		//dali: configure the ephy param.
		rtl8125_powerup_pll();
		rtl8169_init_phy();			//dali: gphy(/gphy ocp) param, disable eee_phy
	}

	/* if TBI is not endbled */
	tpc->cur_page = 0x0;
	if (!(RTL_R16(PHYstatus) & TBI_Enable)) {
		int val = mdio_read(PHY_AUTO_NEGO_REG);

		option = (board_idx >= MAX_UNITS) ? 0 : media[board_idx];
		/* Force RTL8169 in 10/100/1000 Full/Half mode. */
		if (option > 0) {
#ifdef DEBUG_RTL8169
			printf("%s: Force-mode Enabled.\n", name);
#endif
			Cap10_100 = 0, Cap1000 = 0;
			switch (option) {
			case _10_Half:
				Cap10_100 = PHY_Cap_10_Half;
				Cap1000 = PHY_Cap_Null;
				break;
			case _10_Full:
				Cap10_100 = PHY_Cap_10_Full;
				Cap1000 = PHY_Cap_Null;
				break;
			case _100_Half:
				Cap10_100 = PHY_Cap_100_Half;
				Cap1000 = PHY_Cap_Null;
				break;
			case _100_Full:
				Cap10_100 = PHY_Cap_100_Full;
				Cap1000 = PHY_Cap_Null;
				break;
			case _1000_Full:
				Cap10_100 = PHY_Cap_Null;
				Cap1000 = PHY_Cap_1000_Full;
				break;
			default:
				break;
			}
			mdio_write(PHY_AUTO_NEGO_REG, Cap10_100 | (val & 0x1F));	/* leave PHY_AUTO_NEGO_REG bit4:0 unchanged */
			mdio_write(PHY_1000_CTRL_REG, Cap1000);
		} else {
#ifdef DEBUG_RTL8169
			printf("%s: Auto-negotiation Enabled.\n",
			       name);
#endif
			/* enable 10/100 Full/Half Mode, leave PHY_AUTO_NEGO_REG bit4:0 unchanged */
			mdio_write(PHY_AUTO_NEGO_REG,
				   PHY_Cap_10_Half | PHY_Cap_10_Full |
				   PHY_Cap_100_Half | PHY_Cap_100_Full |
				   (val & 0x1F));

			/* enable 1000 Full Mode */
			mdio_write(PHY_1000_CTRL_REG, PHY_Cap_1000_Full);

		}

		/* Enable auto-negotiation and restart auto-nigotiation */
		mdio_write(PHY_CTRL_REG,
			   PHY_Enable_Auto_Nego | PHY_Restart_Auto_Nego);
		udelay(100);

		/* wait for auto-negotiation process */
		for (i = 10000; i > 0; i--) {
			/* check if auto-negotiation complete */
			if (mdio_read(PHY_STAT_REG) & PHY_Auto_Nego_Comp) {
				udelay(100);
				option = RTL_R8(PHYstatus);
				if (option & _1000bpsF) {
#ifdef DEBUG_RTL8169
					printf("%s: 1000Mbps Full-duplex operation.\n",
					       name);
#endif
				} else {
#ifdef DEBUG_RTL8169
					printf("%s: %sMbps %s-duplex operation.\n",
					       name,
					       (option & _100bps) ? "100" :
					       "10",
					       (option & FullDup) ? "Full" :
					       "Half");
#endif
				}
				break;
			} else {
				udelay(100);
			}
		}		/* end for-loop to wait for auto-negotiation process */

	} else {
		udelay(100);
#ifdef DEBUG_RTL8169
		printf
		    ("%s: 1000Mbps Full-duplex operation, TBI Link %s!\n",
		     name,
		     (RTL_R32(TBICSR) & TBILinkOK) ? "OK" : "Failed");
#endif
	}


	tpc->RxDescArray = rtl_alloc_descs(NUM_RX_DESC);
	if (!tpc->RxDescArray)
		return -ENOMEM;

	tpc->TxDescArray = rtl_alloc_descs(NUM_TX_DESC);
	if (!tpc->TxDescArray)
		return -ENOMEM;

	return 0;
}

#ifndef CONFIG_DM_ETH
int rtl8169_initialize(struct bd_info *bis)
{
	pci_dev_t devno;
	int card_number = 0;
	struct eth_device *dev;
	u32 iobase;
	int idx=0;

	while(1){
		unsigned int region;
		u16 device;
		int err;

		/* Find RTL8169 */
		if ((devno = pci_find_devices(supported, idx++)) < 0)
			break;

		pci_read_config_word(devno, PCI_DEVICE_ID, &device);
		switch (device) {
		case 0x8168:
			region = 2;
			break;

		default:
			region = 1;
			break;
		}

		pci_read_config_dword(devno, PCI_BASE_ADDRESS_0 + (region * 4), &iobase);
		iobase &= ~0xf;

		debug ("rtl8169: REALTEK RTL8169 @0x%x\n", iobase);

		dev = (struct eth_device *)malloc(sizeof *dev);
		if (!dev) {
			printf("Can not allocate memory of rtl8169\n");
			break;
		}

		memset(dev, 0, sizeof(*dev));
		sprintf (dev->name, "RTL8169#%d", card_number);

		dev->priv = (void *)(unsigned long)devno;
		dev->iobase = (int)pci_mem_to_phys(devno, iobase);

		dev->init = rtl_reset;
		dev->halt = rtl_halt;
		dev->send = rtl_send;
		dev->recv = rtl_recv;

		err = rtl_init(dev->iobase, dev->name, dev->enetaddr);
		if (err < 0) {
			printf(pr_fmt("failed to initialize card: %d\n"), err);
			free(dev);
			continue;
		}

		eth_register (dev);

		card_number++;
	}
	return card_number;
}
#endif

#ifdef CONFIG_DM_ETH
static int rtl8169_eth_probe(struct udevice *dev)
{

	#ifdef RK_v201709
		struct pci_child_platdata *pplat = dev_get_parent_platdata(dev);
	#else
		struct pci_child_plat *pplat = dev_get_parent_plat(dev);
	#endif

	struct rtl8169_private *priv = dev_get_priv(dev);

	#ifdef RK_v201709
		struct eth_pdata *plat = dev_get_platdata(dev);
	#else
		struct eth_pdata *plat = dev_get_plat(dev);
	#endif

	int region;
	int ret;

	switch (pplat->device) {
	case 0x8125:
	case 0x8161:
	case 0x8168:
		region = 2;
		break;
	default:
		region = 1;
		break;
	}
	
#ifdef RK_v201709
	u32 iobase;
	dm_pci_read_config32(dev, PCI_BASE_ADDRESS_0 + region * 4, &iobase);
	printf("dm_pci_read_config32 region = %d, bar_addr = %x, iobase = %x\n", region, PCI_BASE_ADDRESS_0 + region*4, iobase);
	iobase &= ~0xf;
	priv->iobase = (ulong)dm_pci_mem_to_phys(dev, iobase);
	/* priv->iobase is ulong type, which is 64bit length. and int type is 32bit. we should check the mapped memory address whether 64bit or not.*/	
#else		
	/* priv->iobase is ulong type, which is 64bit length. and int type is 32bit. we should check the mapped memory address whether 64bit or not.*/
	priv->iobase = (ulong)dm_pci_map_bar(dev,
					     PCI_BASE_ADDRESS_0 + region * 4,
					     0, 0,
					     PCI_REGION_TYPE, PCI_REGION_MEM);
#endif

	debug("rtl8169: REALTEK RTL8169 @0x%lx\n", priv->iobase);
	ret = rtl_init(priv->iobase, dev->name, plat->enetaddr);
	if (ret < 0) {
		printf(pr_fmt("failed to initialize card: %d\n"), ret);
		return ret;
	}

	/*
	 * WAR for DHCP failure after rebooting from kernel.
	 * Clear RxDv_Gated_En bit which was set by kernel driver.
	 * Without this, U-Boot can't get an IP via DHCP.
	 * Register (FuncEvent, aka MISC) and RxDv_Gated_En bit are from
	 * the r8169.c kernel driver.
	 */

	u32 val = RTL_R32(FuncEvent);
	debug("%s: FuncEvent/Misc (0xF0) = 0x%08X\n", __func__, val);
	val &= ~RxDv_Gated_En;
	RTL_W32(FuncEvent, val);

	return 0;
}

static const struct eth_ops rtl8169_eth_ops = {
	.start	= rtl8169_eth_start,
	.send	= rtl8169_eth_send,
	.recv	= rtl8169_eth_recv,
	.stop	= rtl8169_eth_stop,
	.write_hwaddr = rtl8169_write_hwaddr,
};

static const struct udevice_id rtl8169_eth_ids[] = {
	{ .compatible = "realtek,rtl8169" },
	{ }
};

U_BOOT_DRIVER(eth_rtl8169) = {
	.name	= "eth_rtl8169",
	.id	= UCLASS_ETH,
	.of_match = rtl8169_eth_ids,
	.probe	= rtl8169_eth_probe,
	.ops	= &rtl8169_eth_ops,
	#ifdef RK_v201709
		.priv_auto_alloc_size = sizeof(struct rtl8169_private),
		.platdata_auto_alloc_size = sizeof(struct eth_pdata),
	#else
		.priv_auto	= sizeof(struct rtl8169_private),
		.plat_auto	= sizeof(struct eth_pdata),
	#endif
};

U_BOOT_PCI_DEVICE(eth_rtl8169, supported);
#endif
