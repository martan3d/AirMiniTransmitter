// The LAST entry is active!
// For transmitter
#define TRANSMIT
// For receiver
#undef TRANSMIT

// The LAST entry is active!
// For other transceiver boards that operate at 26MHz
#undef ANAREN
// For Anaren transceiver boards that operate at 27MHz
#define ANAREN

// The LAST entry is active!
// For CE-approved use
#undef FCC_IC_APPROVED
// For US/Canadian use
#define FCC_IC_APPROVED


// Determined from defines above
// Define whether is or is NOT operating @27 MHz
#ifdef ANAREN
#define TWENTY_SEVEN_MHZ
#else
#undef  TWENTY_SEVEN_MHZ
#endif

// Explict RECEIVE define/undef
#ifdef TRANSMIT
#undef RECEIVE
#else
#define RECEIVE
#endif

// Explicit CE undef/define
#ifdef FCC_IC_APPROVED
#undef CE_APPROVED
#else
#define CE_APPROVED
#endif
