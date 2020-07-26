// Rf settings for CC2500
RF_SETTINGS code rfSettings = {
    0x2E,  // IOCFG2              GDO2Output Pin Configuration 
    0x2E,  // IOCFG1              GDO1Output Pin Configuration 
    0x0D,  // IOCFG0              GDO0Output Pin Configuration 
    0x47,  // FIFOTHR             RX FIFO and TX FIFO Thresholds
    0xD3,  // SYNC1               Sync Word, High Byte 
    0x91,  // SYNC0               Sync Word, Low Byte 
    0xFF,  // PKTLEN              Packet Length 
    0x04,  // PKTCTRL1            Packet Automation Control
    0x32,  // PKTCTRL0            Packet Automation Control
    0x00,  // ADDR                Device Address 
    0x00,  // CHANNR              Channel Number 
    0x06,  // FSCTRL1             Frequency Synthesizer Control 
    0x00,  // FSCTRL0             Frequency Synthesizer Control 
    0x5D,  // FREQ2               Frequency Control Word, High Byte 
    0x93,  // FREQ1               Frequency Control Word, Middle Byte 
    0xB1,  // FREQ0               Frequency Control Word, Low Byte 
    0xCA,  // MDMCFG4             Modem Configuration 
    0x93,  // MDMCFG3             Modem Configuration 
    0x00,  // MDMCFG2             Modem Configuration
    0x22,  // MDMCFG1             Modem Configuration
    0xF7,  // MDMCFG0             Modem Configuration 
    0x50,  // DEVIATN             Modem Deviation Setting 
    0x07,  // MCSM2               Main Radio Control State Machine Configuration 
    0x30,  // MCSM1               Main Radio Control State Machine Configuration
    0x18,  // MCSM0               Main Radio Control State Machine Configuration 
    0x16,  // FOCCFG              Frequency Offset Compensation Configuration
    0x6C,  // BSCFG               Bit Synchronization Configuration
    0x03,  // AGCCTRL2            AGC Control
    0x40,  // AGCCTRL1            AGC Control
    0x91,  // AGCCTRL0            AGC Control
    0x87,  // WOREVT1             High Byte Event0 Timeout 
    0x6B,  // WOREVT0             Low Byte Event0 Timeout 
    0xF8,  // WORCTRL             Wake On Radio Control
    0x56,  // FREND1              Front End RX Configuration 
    0x10,  // FREND0              Front End TX configuration 
    0xA9,  // FSCAL3              Frequency Synthesizer Calibration 
    0x0A,  // FSCAL2              Frequency Synthesizer Calibration 
    0x00,  // FSCAL1              Frequency Synthesizer Calibration 
    0x11,  // FSCAL0              Frequency Synthesizer Calibration 
    0x41,  // RCCTRL1             RC Oscillator Configuration 
    0x00,  // RCCTRL0             RC Oscillator Configuration 
    0x59,  // FSTEST              Frequency Synthesizer Calibration Control 
    0x7F,  // PTEST               Production Test 
    0x3F,  // AGCTEST             AGC Test 
    0x88,  // TEST2               Various Test Settings 
    0x31,  // TEST1               Various Test Settings 
    0x0B,  // TEST0               Various Test Settings 
    0x80,  // PARTNUM             Chip ID 
    0x03,  // VERSION             Chip ID 
    0x00,  // FREQEST             Frequency Offset Estimate from Demodulator 
    0x00,  // LQI                 Demodulator Estimate for Link Quality 
    0x00,  // RSSI                Received Signal Strength Indication 
    0x00,  // MARCSTATE           Main Radio Control State Machine State 
    0x00,  // WORTIME1            High Byte of WOR Time 
    0x00,  // WORTIME0            Low Byte of WOR Time 
    0x00,  // PKTSTATUS           Current GDOxStatus and Packet Status 
    0x00,  // VCO_VC_DAC          Current Setting from PLL Calibration Module 
    0x00,  // TXBYTES             Underflow and Number of Bytes 
    0x00,  // RXBYTES             Underflow and Number of Bytes 
    0x00,  // RCCTRL1_STATUS      Last RC Oscillator Calibration Result 
    0x00,  // RCCTRL0_STATUS      Last RC Oscillator Calibration Result 
};