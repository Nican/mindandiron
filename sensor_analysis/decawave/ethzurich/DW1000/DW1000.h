#ifndef DW1000_H
#define DW1000_H

// register addresses
//      Mnemonic                    Address Bytes Description
#define DW1000_DEV_ID               0x00 //     4 Device Identifier – includes device type and revision information
#define DW1000_EUI                  0x01 //     8 Extended Unique Identifier
#define DW1000_PANADR               0x03 //     4 PAN Identifier and Short Address
#define DW1000_SYS_CFG              0x04 //     4 System Configuration bitmap
#define DW1000_SYS_TIME             0x06 //     5 System Time Counter (40-bit)
#define DW1000_TX_FCTRL             0x08 //     5 Transmit Frame Control
#define DW1000_TX_BUFFER            0x09 //  1024 Transmit Data Buffer
#define DW1000_DX_TIME              0x0A //     5 Delayed Send or Receive Time (40-bit)
#define DW1000_RX_FWTO              0x0C //     2 Receive Frame Wait Timeout Period
#define DW1000_SYS_CTRL             0x0D //     4 System Control Register
#define DW1000_SYS_MASK             0x0E //     4 System Event Mask Register
#define DW1000_SYS_STATUS           0x0F //     5 System Event Status Register
#define DW1000_RX_FINFO             0x10 //     4 RX Frame Information                (in double buffer set)
#define DW1000_RX_BUFFER            0x11 //  1024 Receive Data Buffer                 (in double buffer set)
#define DW1000_RX_FQUAL             0x12 //     8 Rx Frame Quality information        (in double buffer set)
#define DW1000_RX_TTCKI             0x13 //     4 Receiver Time Tracking Interval     (in double buffer set)
#define DW1000_RX_TTCKO             0x14 //     5 Receiver Time Tracking Offset       (in double buffer set)
#define DW1000_RX_TIME              0x15 //    14 Receive Message Time of Arrival     (in double buffer set)
#define DW1000_TX_TIME              0x17 //    10 Transmit Message Time of Sending    (in double buffer set)
#define DW1000_TX_ANTD              0x18 //     2 16-bit Delay from Transmit to Antenna
#define DW1000_SYS_STATE            0x19 //     5 System State information
#define DW1000_ACK_RESP_T           0x1A //     4 Acknowledgement Time and Response Time
#define DW1000_RX_SNIFF             0x1D //     4 Pulsed Preamble Reception Configuration
#define DW1000_TX_POWER             0x1E //     4 TX Power Control
#define DW1000_CHAN_CTRL            0x1F //     4 Channel Control
#define DW1000_USR_SFD              0x21 //    41 User-specified short/long TX/RX SFD sequences
#define DW1000_AGC_CTRL             0x23 //    32 Automatic Gain Control configuration
#define DW1000_EXT_SYNC             0x24 //    12 External synchronisation control.
#define DW1000_ACC_MEM              0x25 //  4064 Read access to accumulator data
#define DW1000_GPIO_CTRL            0x26 //    44 Peripheral register bus 1 access - GPIO control
#define DW1000_DRX_CONF             0x27 //    44 Digital Receiver configuration
#define DW1000_RF_CONF              0x28 //    58 Analog RF Configuration
#define DW1000_TX_CAL               0x2A //    52 Transmitter calibration block
#define DW1000_FS_CTRL              0x2B //    21 Frequency synthesiser control block
#define DW1000_AON                  0x2C //    12 Always-On register set
#define DW1000_OTP_IF               0x2D //    18 One Time Programmable Memory Interface
#define DW1000_LDE_CTRL             0x2E //     - Leading edge detection control block
#define DW1000_DIG_DIAG             0x2F //    41 Digital Diagnostics Interface
#define DW1000_PMSC                 0x36 //    48 Power Management System Control Block
 
#define DW1000_WRITE_FLAG           0x80 // First Bit of the address has to be 1 to indicate we want to write
#define DW1000_SUBADDRESS_FLAG      0x40 // if we have a sub address second Bit has to be 1
#define DW1000_2_SUBADDRESS_FLAG    0x80 // if we have a long sub adress (more than 7 Bit) we set this Bit in the first part

#endif