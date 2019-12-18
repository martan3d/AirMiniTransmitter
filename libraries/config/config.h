// The LAST entry is active!
// For transmitter
#define TRANSMIT
// For receiver
#undef TRANSMIT

// For Anaren transeiver boards that operate at 27MHz
#define ANAREN
// For other transeiver boards that operate at 26MHz
#undef ANAREN

// Define whether is or is NOT operating @27 MHz
#ifdef ANAREN
#define TWENTY_SEVEN_MHZ
#else
#undef  TWENTY_SEVEN_MHZ
#endif
