#! /opt/local/bin/gawk -f
# Created by: Darrell R. Lamm, darrelllamm0@gmail.com
# Purpose: Compute the band width and baud rate for the CC1101 using MDMCFG4 and MDMCFG3
# Note: You must use gawk, not the garden-variety version of awk
BEGIN {
   entry_line = "Enter MDMCFG4 MDMCFG3 MDMCFG2 MDMCFG1 MDMCFG0 fXOSC:"
   print entry_line;
}

{
   gsub(/,/," ");
}

NF < 6 {print; next;}

{
   MDMCFG4 = strtonum($1); # for conversion of hexstr
   MDMCFG3 = strtonum($2); # for conversion of hexstr
   MDMCFG2 = strtonum($3);
   MDMCFG1 = strtonum($4);
   MDMCFG0 = strtonum($5);
   fXOSC = $6;
   $1 = $2 = $3 = $4 = $5 = $6 = "";
   print "Comment " $0;

   MDMCFG4h = hexstr(MDMCFG4);
   MDMCFG3h = hexstr(MDMCFG3);
   MDMCFG2h = hexstr(MDMCFG2);
   MDMCFG1h = hexstr(MDMCFG1);
   MDMCFG0h = hexstr(MDMCFG0);

   print "MDMCFG4: " MDMCFG4h " MDMCFG3: " MDMCFG3h " MDMCFG2: " MDMCFG2h " MDMCFG1: " MDMCFG1h " MDMCFG0: " MDMCFG0h " fXOSC: " fXOSC;;

   CHANBW_E = convert(MDMCFG4,7,6);
   CHANBW_M = convert(MDMCFG4,5,4);
   DRATE_E =  convert(MDMCFG4,3,0);
   DRATE_M = convert(MDMCFG3,7,0);
   CHANBW_Eh = hexstr(CHANBW_E);
   CHANBW_Mh = hexstr(CHANBW_M);
   DRATE_Eh = hexstr(DRATE_E);
   DRATE_Mh = hexstr(DRATE_M);
   print "CHANBW_E: " CHANBW_Eh " CHANBW_M: " CHANBW_Mh " fXOSC: " fXOSC " bandwidth: " bandwidth(CHANBW_E, CHANBW_M, fXOSC) " Hz";
   print "DRATE_E: " DRATE_Eh " DRATE_M: " DRATE_Mh " fXOSC: " fXOSC " baudrate: " baud(DRATE_E, DRATE_M, fXOSC) " baud";

   SYNC_MODE = convert(MDMCFG2,2,0);
   print "SYNC_MODE: " bits2str(SYNC_MODE,3);
   MANCHESTER_EN = convert(MDMCFG2,3,3);
   print "MANCHESTER_EN: " bits2str(MANCHESTER_EN,1);
   MOD_FORMAT = convert(MDMCFG2,6,4);
   print "MOD_FORMAT: " bits2str(MOD_FORMAT,3);
   DEM_DCFILT_OFF = convert(MDMCFG2,7,7);
   print "DEM_DCFILT_OFF: " bits2str(DEM_DCFILT_OFF,1); 

   FEC_EN = convert(MDMCFG1,7,7);
   print "FEC_EN: " bits2str(FEC_EN,1);

   NUM_PREAMBLE = convert(MDMCFG1,6,4);
   print "NUM_PREAMBLE: " bits2str(NUM_PREAMBLE,3);

   CHANSPC_E = convert(MDMCFG1,1,0);
   CHANSPC_Eh = hexstr(CHANSPC_E);
   CHANSPC_M = convert(MDMCFG0,7,0);
   CHANSPC_Mh = hexstr(CHANSPC_M);
   print "CHANSPC_E: " CHANSPC_Eh " CHANSPC_M: " CHANSPC_Mh " Channel spacing: " channel_spacing(CHANSPC_E,CHANSPC_M,fXOSC) " Hz";
   print "---------------";
   
}

{
   print entry_line;
}

function channel_spacing(CHANSPC_E,CHANSPC_M,fXOSC) {
   return fXOSC*(256+CHANSPC_M)*(2**CHANSPC_E)/(2**18);
}

function bandwidth(CHANBW_E,CHANBW_M,fXOSC) {
   return fXOSC/(8*(4+CHANBW_M))/2**CHANBW_E;
}

function baud(DRATE_E, DRATE_M, fXOSC) {
   return fXOSC*(256+DRATE_M)*(2**DRATE_E)/(2**28);
}

function bits2str(bits, width,        data)
{
    if (!length(width)) width = 8;
    if (bits != 0) {
       for (; bits != 0; bits = rshift(bits, 1))
           data = (and(bits, 1) ? "1" : "0") data;
    }
    else data = "0";

    while ((length(data) % width) != 0)
        data = "0" data

    return data
}

function convert(val, high, low) {
   return and(2**(high-low+1)-1, rshift(val,low));
}

function hexstr(val) {
   return sprintf("0x%X",val);
}
