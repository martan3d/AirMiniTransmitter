// The LAST entry is active!
// For transmitter
#define TRANSMIT
// For receiver
#undef TRANSMIT

// The LAST entry is active!
// For 26MHz transceiver
#undef TWENTY_SEVEN_MHZ
// For 27MHz transceivers (e.g., Anaren)
#define TWENTY_SEVEN_MHZ

// The LAST entry is active!
// For CE-approved use
#undef FCC_IC_APPROVED
// For US/Canadian use
#define FCC_IC_APPROVED



////////////////////////////////
// Determined from defines above
// Do NOT edit below this line
////////////////////////////////

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

// Explicit Frequency
#ifdef TWENTY_SEVEN_MHZ
#undef TWENTY_SIX_MHZ
#else
#define TWENTY_SIX_MHZ
#endif

// Messages
#ifdef TRANSMIT
#warning "Note: Compiling for Transmitter"
#endif
#ifdef RECEIVE
#warning "Note: Compiling for Receiver"
#endif

#ifdef TWENTY_SEVEN_MHZ
#warning "Note: Compiling for 27MHz transceivers"
#endif
#ifdef TWENTY_SIX_MHZ
#warning "Note: Compiling for 26MHz transceivers"
#endif

#ifdef FCC_IC_APPROVED
#warning "Note: Compiling for North American use"
#endif
#ifdef CE_APPROVED
#warning "Note: Compiling for European use"
#endif

