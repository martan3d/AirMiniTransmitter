#! /opt/local/bin/gawk -f
# Created by: Darrell R. Lamm, darrelllamm0@gmail.com
# Purpose: Compute frequency from FREQ[210]
# Note: You must use gawk, not the garden-variety version of awk
BEGIN {
   print "Enter FREQ2 FREQ1 FREQ0 fXOSC:"
}

{
   gsub(/,/," ");
}

NF < 4 {print; next;}

{
   FREQ2 = strtonum($1); # for conversion of hex
   FREQ1 = strtonum($2); # for conversion of hex
   FREQ0 = strtonum($3); # for conversion of hex
   fXOSC = $4;
   $1 = $2= $3 = $4 = "";
   print "Comment:" $0;

   FREQ2Shifted = lshift(and(FREQ2,0x3F),16);
   FREQ1Shifted = lshift(and(FREQ1,0xFF),8);
   FREQ0Shifted = lshift(and(FREQ0,0xFF),0);
   FREQ = FREQ2Shifted + FREQ1Shifted + FREQ0Shifted;

   FREQ2h = sprintf("0x%X",FREQ2);
   FREQ1h = sprintf("0x%X",FREQ1);
   FREQ0h = sprintf("0x%X",FREQ0);
   FREQh = sprintf("0x%X",FREQ);
   print "FREQ2: " FREQ2h " FREQ1: " FREQ1h " FREQ0: " FREQ0h " fXOSC: " fXOSC;
   #print "FREQ: " FREQh " fcarrier: " fcarrier(FREQ,fXOSC)/1.0E6 " MHz";
   printf("%s %s %s %3.6f %s\n", "FREQ: ", FREQh, " fcarrier: ", fcarrier(FREQ,fXOSC)/1.0E6, " MHz");
   print "---------------";
}

{
   print "Enter FREQ2 FREQ1 FREQ0 fXOSC:"
}
function fcarrier(FREQ,fXOSC) {
   return fXOSC*FREQ/2**16;
}

