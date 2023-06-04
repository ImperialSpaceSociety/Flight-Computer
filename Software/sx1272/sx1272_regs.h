
enum sx1272_common_regs {

  SxRegFifo = 0x00,
  SxRegOpMode,
  SxRegFrfMsb,
  SxRegFrfMib,
  SxRegFrfLsb,
  SxRegPaConfig,
  SxRegPaRamp,
  SxRegOcp,
  SxRegLna,

  SxRegDioMapping1 = 0x40,
  SxRegDioMapping2,
  SxRegVersion,
};

enum sx1272_regs_lora {

  SxLoraRegFifoAddrPtr = 0x0D,
  SxLoraRegFifoTxBaseAddr,
  SxLoraRegFifoRxBaseAddr,
  SxLoraRegFifoRxCurrentAddr,
  SxLoraRegIrqFlagsMask,
  SxLoraRegIrqFlags,
  SxLoraRegRxNbBytes,
  SxLoraRegRxHeaderCntValueMsb,
  SxLoraRegRxHeaderCntValueLsb,
  SxLoraRegRxPacketCntValueMsb,
  SxLoraRegRxPacketCntValueLsb,
  SxLoraRegModemStat,
  SxLoraRegPktSnrValue,
  SxLoraRegPktRssiValue,
  SxLoraRegRssiValue,
  SxLoraRegHopChannel,
  SxLoraRegModemConfig1,
  SxLoraRegModemConfig2,
  SxLoraRegSymbTimeoutLsb,
  SxLoraRegPreambleMsb,
  SxLoraRegPreambleLsb,
  SxLoraRegPayloadLength,
  SxLoraRegMaxPayloadLength,
  SxLoraRegHopPeriod,
  SxLoraRegFifoRxByteAddr,

  SxLoraRegRssiWideband = 0x2C,

  SxLoraRegDetectOptimize = 0x31,

  SxLoraRegDetectionThreshold = 0x37,

  SxLoraRegSyncWord = 0x39,
};
