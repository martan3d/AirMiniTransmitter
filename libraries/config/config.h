// The LAST entry is active!
// For transmitter
#define TRANSMIT
// For receiver
#undef TRANSMIT

// The LAST entry is active!
// For Anaren transceiver boards that operate at 27MHz
#define ANAREN
// For other transceiver boards that operate at 26MHz
#undef ANAREN

// Determined from defines above
// Define whether is or is NOT operating @27 MHz
#ifdef ANAREN
#define TWENTY_SEVEN_MHZ
#else
#undef  TWENTY_SEVEN_MHZ
#endif
